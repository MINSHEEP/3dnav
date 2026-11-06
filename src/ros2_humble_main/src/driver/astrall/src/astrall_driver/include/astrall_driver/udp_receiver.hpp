// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__UDP_RECEIVER_HPP_
#define ASTRALL_DRIVER__UDP_RECEIVER_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

namespace astrall_driver {

struct UdpReceiverSnapshot {
  bool running_ = false;
  uint16_t port_ = 0;
  uint64_t datagrams_ = 0;
  uint64_t truncated_ = 0;
  uint64_t errors_ = 0;
  std::string last_error_;
};

/**
 * @brief POSIX IPv4接收线程；poll+唤醒管道，停止不依赖新UDP包，不启用端口复用。
 * Start/Stop/析构须外部串行化且不能从接收回调调用；Snapshot可并发。
 * 回调仅在本线程同步借用数据，须快速有界；异常记录后停止接收，不静默吞掉。
 */
class UdpReceiver {
 public:
  using Callback = std::function<void (const uint8_t*, size_t)>;
  UdpReceiver() = default;
  ~UdpReceiver();
  /** @brief 0端口由OS分配供离线测试；失败返回false并保留具体socket诊断。 */
  bool Start(const std::string& address, uint16_t port, size_t max_size,
      Callback callback);
  /** @brief 唤醒、join后关闭socket和管道；可重复，正常停止不记错误。 */
  void Stop();
  /** @brief 真实接收次数、截断/线程错误与OS实际绑定端口。 */
  UdpReceiverSnapshot Snapshot() const;

 private:
  void Run(size_t max_size, const Callback& callback);
  void Error(const std::string& message);
  void CloseDescriptors();
  int socket_ = -1;
  int wake_read_ = -1;
  int wake_write_ = -1;
  std::atomic<bool> stop_{ true };
  std::atomic<bool> running_{ false };
  std::thread thread_;
  mutable std::mutex mutex_;
  UdpReceiverSnapshot snapshot_;
};

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__UDP_RECEIVER_HPP_
