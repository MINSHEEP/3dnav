// Copyright 2026 Hypertron ROS 2 contributors

#include "sdk_callback_dispatcher.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <deque>
#include <utility>

namespace astrall_driver {
namespace {

constexpr size_t kMaxPendingCallbacks = 256;

template <typename T>
bool CopyValue(void* data, uint16_t length,
    SdkCallbackDispatcher::Value* value) {
  if (data == nullptr || length != sizeof(T)) {
    return false;
  }
  T copy{};
  std::memcpy(&copy, data, sizeof(copy));
  *value = copy;
  return true;
}

bool Decode(SdkCallbackDispatcher::Kind kind, void* data, uint16_t length,
    SdkCallbackDispatcher::Value* value) {
  using Kind = SdkCallbackDispatcher::Kind;
  switch (kind) {
    case Kind::kHeartbeat: return length == 0;
    case Kind::kStatus: return CopyValue<AstrallSdkStatus>(data, length, value);
    case Kind::kImu: return CopyValue<AstrallImuData>(data, length, value);
    case Kind::kSport: return CopyValue<AstrallSportData>(data, length, value);
    case Kind::kJoystick: return CopyValue<AstrallJoystickData>(
        data, length, value);
  }
  return false;
}

}  // namespace

struct SdkCallbackDispatcher::State {
  std::mutex mutex_;
  std::condition_variable condition_;
  bool stopped_ = false;
  bool dispatching_ = false;
  std::thread::id worker_id_;
  std::deque<std::pair<std::weak_ptr<Slot>, Value>> queue_;
  std::vector<std::weak_ptr<Slot>> slots_;
  CallbackStatistics statistics_;
};

SdkCallbackDispatcher::SdkCallbackDispatcher()
  : state_(std::make_shared<State>()),
  worker_([state = state_] {
    Run(state);
  }) {}

SdkCallbackDispatcher::~SdkCallbackDispatcher() {
  auto retired = Disable();
  if (worker_.get_id() == std::this_thread::get_id()) {
    // 用户可在回调内释放客户端；线程仅捕获 State，不访问已析构的 this。
    worker_.detach();
    return;
  }
  worker_.join();
}

AstrallConfig SdkCallbackDispatcher::MakeConfig(SdkCallbacks callbacks) {
  auto heartbeat = Add([callback = std::move(callbacks.heartbeat_)](
            const Value&) {
      if (callback) {
        callback();
      }
    });
  auto status = Add([callback = std::move(callbacks.status_)](
            const Value& value) {
      if (callback) {
        callback(std::get<AstrallSdkStatus>(value));
      }
    });
  config_slots_ = { heartbeat, status };
  return { Bind(heartbeat, Kind::kHeartbeat), Bind(status, Kind::kStatus) };
}

SdkCallbackDispatcher::Handle SdkCallbackDispatcher::Add(
  std::function<void(const Value&)> callback) {
  auto slot = std::make_shared<Slot>();
  slot->callback_ = std::make_shared<const Callback>(std::move(callback));
  std::lock_guard<std::mutex> lock(state_->mutex_);
  auto& slots = state_->slots_;
  slots.erase(std::remove_if(slots.begin(), slots.end(),
      [](const auto& item) {
      return item.expired();
    }), slots.end());
  slots.push_back(slot);
  return slot;
}

RawSdkCallback SdkCallbackDispatcher::Bind(const Handle& slot, Kind kind) {
  std::weak_ptr<State> weak_state = state_;
  std::weak_ptr<Slot> weak_slot = slot;
  return [weak_state, weak_slot, kind](void* data, uint16_t length) {
           auto state = weak_state.lock();
           auto current_slot = weak_slot.lock();
           if (!state || !current_slot) {
             return;
           }
           std::lock_guard<std::mutex> lock(state->mutex_);
           if (state->stopped_) {
             return;
           }
           {
             std::lock_guard<std::mutex> slot_lock(current_slot->mutex_);
             if (!current_slot->enabled_) {
               return;
             }
           }
           // 关闭/取消后不再读取 SDK 的临时缓冲区，先做寿命检查再解码。
           Value value;
           const bool valid = Decode(kind, data, length, &value);
           if (!valid) {
             ++state->statistics_.invalid_payloads_;
             return;
           }
           if (state->queue_.size() >= kMaxPendingCallbacks) {
             ++state->statistics_.dropped_events_;
             return;
           }
           state->queue_.emplace_back(weak_slot, std::move(value));
           state->condition_.notify_one();
  };
}

void SdkCallbackDispatcher::ActivateConfig() {
  for (const auto& slot : config_slots_) {
    Activate(slot);
  }
}

void SdkCallbackDispatcher::Activate(const Handle& slot) {
  std::lock_guard<std::mutex> lock(slot->mutex_);
  slot->ready_ = true;
  slot->condition_.notify_all();
}

SdkCallbackDispatcher::CallbackHandle SdkCallbackDispatcher::Disable(
  const Handle& slot) {
  std::lock_guard<std::mutex> lock(slot->mutex_);
  slot->enabled_ = false;
  slot->ready_ = true;
  auto retired = std::move(slot->callback_);
  slot->condition_.notify_all();
  return retired;
}

void SdkCallbackDispatcher::Drain(const Handle& slot) {
  std::unique_lock<std::mutex> lock(slot->mutex_);
  if (slot->thread_id_ == std::this_thread::get_id()) {
    return;
  }
  slot->condition_.wait(lock, [&slot] {
      return !slot->running_;
    });
}

std::vector<SdkCallbackDispatcher::CallbackHandle> SdkCallbackDispatcher::
Disable() {
  std::vector<Handle> slots;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    state_->stopped_ = true;
    state_->queue_.clear();
    for (const auto& weak_slot : state_->slots_) {
      if (auto slot = weak_slot.lock()) {
        slots.push_back(std::move(slot));
      }
    }
    state_->condition_.notify_all();
  }
  std::vector<CallbackHandle> retired;
  for (const auto& slot : slots) {
    retired.push_back(Disable(slot));
  }
  return retired;
}

void SdkCallbackDispatcher::Drain() {
  std::vector<Handle> slots;
  {
    std::lock_guard<std::mutex> lock(state_->mutex_);
    for (const auto& weak_slot : state_->slots_) {
      if (auto slot = weak_slot.lock()) {
        slots.push_back(std::move(slot));
      }
    }
  }
  for (const auto& slot : slots) {
    Drain(slot);
  }
}

bool SdkCallbackDispatcher::WaitIdle(uint32_t timeout_ms) {
  std::unique_lock<std::mutex> lock(state_->mutex_);
  if (state_->worker_id_ == std::this_thread::get_id()) {
    return false;
  }
  return state_->condition_.wait_for(lock,
             std::chrono::milliseconds(timeout_ms),
             [this] {
      return state_->queue_.empty() && !state_->dispatching_;
    });
}

CallbackStatistics SdkCallbackDispatcher::Statistics() const {
  std::lock_guard<std::mutex> lock(state_->mutex_);
  return state_->statistics_;
}

void SdkCallbackDispatcher::Invoke(const std::shared_ptr<State>& state,
    const Handle& slot, const Value& value) {
  CallbackHandle callback;
  {
    std::unique_lock<std::mutex> lock(slot->mutex_);
    slot->condition_.wait(lock, [&slot] {
        return slot->ready_;
      });
    if (!slot->enabled_) {
      return;
    }
    callback = slot->callback_;
    slot->running_ = true;
    slot->thread_id_ = std::this_thread::get_id();
  }
  try {
    (*callback)(value);
  } catch (...) {
    auto exception = std::current_exception();
    {
      std::lock_guard<std::mutex> lock(state->mutex_);
      ++state->statistics_.callback_errors_;
      state->statistics_.last_exception_.swap(exception);
    }
    // 旧异常可能包含重入统计接口的用户析构逻辑，必须在锁外释放。
  }
  // 捕获资源的销毁也属于回调完成，必须先于 Drain 的完成通知。
  callback.reset();
  std::lock_guard<std::mutex> lock(slot->mutex_);
  slot->running_ = false;
  slot->thread_id_ = {};
  slot->condition_.notify_all();
}

void SdkCallbackDispatcher::Run(const std::shared_ptr<State>& state) {
  std::unique_lock<std::mutex> lock(state->mutex_);
  state->worker_id_ = std::this_thread::get_id();
  while (true) {
    state->condition_.wait(lock,
        [&state] {
        return state->stopped_ || !state->queue_.empty();
      });
    if (state->stopped_) {
      return;
    }
    auto event = std::move(state->queue_.front());
    state->queue_.pop_front();
    state->dispatching_ = true;
    lock.unlock();
    if (auto slot = event.first.lock()) {
      Invoke(state, slot, event.second);
    }
    lock.lock();
    state->dispatching_ = false;
    state->condition_.notify_all();
  }
}

}  // namespace astrall_driver
