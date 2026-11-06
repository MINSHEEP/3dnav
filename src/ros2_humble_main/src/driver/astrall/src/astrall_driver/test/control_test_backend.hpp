// Copyright 2026 Hypertron ROS 2 contributors

#ifndef CONTROL_TEST_BACKEND_HPP_
#define CONTROL_TEST_BACKEND_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "astrall_driver/velocity_controller.hpp"
#include "sdk_test_backend.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace test {

struct MoveEntry {
  std::chrono::steady_clock::time_point time_;
  VelocityCommand command_;
};

struct TextEntry {
  std::chrono::steady_clock::time_point time_;
  std::string text_;
};

/** @brief 记录实际原始 SDK 入口/写入，不以驱动内部计数代替时序证据。 */
struct ControlBackend {
  std::shared_ptr<Backend> base_ = std::make_shared<Backend>();
  std::mutex mutex_;
  std::vector<MoveEntry> moves_;
  std::vector<TextEntry> texts_;
  std::vector<std::chrono::steady_clock::time_point> hearts_;
  std::vector<uint16_t> modes_;
  std::vector<uint16_t> lights_;
  std::atomic<uint16_t> actual_{ ASTRALL_SPORT_STATUS_MOVE };
  std::atomic<uint16_t> mode_result_{ ASTRALL_RES_SUCCESSED };
  std::atomic<uint16_t> move_result_{ ASTRALL_RES_SUCCESSED };

  std::vector<MoveEntry> Moves() {
    std::lock_guard<std::mutex> lock(mutex_);
    return moves_;
  }

  std::vector<TextEntry> Texts() {
    std::lock_guard<std::mutex> lock(mutex_);
    return texts_;
  }
};

inline SdkFunctions ControlFunctions(
  const std::shared_ptr<ControlBackend>& trace) {
  auto functions = Functions(trace->base_);
  functions.get_sport_ = [trace] {
        return trace->actual_.load();
      };
  functions.move_ = [trace](float x, float y, float yaw, uint32_t timeout) {
        EXPECT_LE(timeout, 5U);
        std::lock_guard<std::mutex> lock(trace->mutex_);
        trace->moves_.push_back(
          { std::chrono::steady_clock::now(), { x, y, yaw } });
        return trace->move_result_.load();
      };
  functions.heartbeat_ = [trace](uint32_t timeout) {
        EXPECT_LE(timeout, 5U);
        std::lock_guard<std::mutex> lock(trace->mutex_);
        trace->hearts_.push_back(std::chrono::steady_clock::now());
        return ASTRALL_RES_SUCCESSED;
      };
  functions.mode_ = [trace](AstrallSportModeCmd command, uint32_t timeout) {
        EXPECT_LE(timeout, 5U);
        std::lock_guard<std::mutex> lock(trace->mutex_);
        trace->modes_.push_back(command);
        return trace->mode_result_.load();
      };
  functions.light_ = [trace](AstrallLightCmd command, uint32_t) {
        std::lock_guard<std::mutex> lock(trace->mutex_);
        trace->lights_.push_back(command);
        return ASTRALL_RES_SUCCESSED;
      };
  functions.send_message_ = [trace](char* data, uint16_t length) {
        std::lock_guard<std::mutex> lock(trace->mutex_);
        trace->texts_.push_back({ std::chrono::steady_clock::now(),
                                  std::string(data, length) });
        return ASTRALL_RES_SUCCESSED;
      };
  return functions;
}

}  // namespace test
}  // namespace astrall_driver

#endif  // CONTROL_TEST_BACKEND_HPP_
