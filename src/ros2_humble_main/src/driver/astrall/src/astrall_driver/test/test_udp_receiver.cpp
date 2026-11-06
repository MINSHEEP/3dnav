// Copyright 2026 Hypertron ROS 2 contributors

#include <atomic>
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <thread>

#include "astrall_driver/udp_receiver.hpp"
#include "gtest/gtest.h"
#include "udp_test_sender.hpp"

namespace astrall_driver {
namespace {

using namespace std::chrono_literals;

bool Wait(const std::function<bool()>& predicate) {
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  return predicate();
}

size_t Descriptors() {
  return std::distance(std::filesystem::directory_iterator("/proc/self/fd"),
             std::filesystem::directory_iterator{});
}

TEST(UdpReceiver, RealLoopbackStopRebindAndNoDescriptorLeak) {
  const auto descriptors = Descriptors();
  std::atomic<int> calls{ 0 };
  UdpReceiver receiver;
  uint16_t port = 0;
  for (int repetition = 0; repetition < 5; ++repetition) {
    ASSERT_TRUE(receiver.Start("127.0.0.1", port, 32,
        [&](const uint8_t* data, size_t size) {
          EXPECT_EQ(size, 3U);
          EXPECT_EQ(data[1], 2);
          ++calls;
        })) << receiver.Snapshot().last_error_;
    port = receiver.Snapshot().port_;
    ASSERT_GT(port, 0);
    ASSERT_TRUE(test::SendUdp(port, { 1, 2, 3 }));
    ASSERT_TRUE(Wait([&] {
          return calls > repetition;
        }));
    const auto start = std::chrono::steady_clock::now();
    receiver.Stop();
    EXPECT_LT(std::chrono::steady_clock::now() - start, 200ms);
    EXPECT_EQ(receiver.Snapshot().errors_, 0U);
    EXPECT_FALSE(receiver.Snapshot().running_);
    receiver.Stop();
  }
  EXPECT_EQ(Descriptors(), descriptors);
}

TEST(UdpReceiver, ExclusiveBindInvalidAddressAndTruncationAreObservable) {
  UdpReceiver receiver;
  EXPECT_FALSE(receiver.Start("not-ip", 0, 8, [](const uint8_t*, size_t) {
      }));
  EXPECT_FALSE(receiver.Snapshot().last_error_.empty());
  std::atomic<int> calls{ 0 };
  ASSERT_TRUE(receiver.Start("127.0.0.1", 0, 8,
      [&](const uint8_t*, size_t) {
        ++calls;
      }));
  UdpReceiver conflict;
  EXPECT_FALSE(conflict.Start("127.0.0.1", receiver.Snapshot().port_, 8,
      [](const uint8_t*, size_t) {
      }));
  ASSERT_TRUE(test::SendUdp(receiver.Snapshot().port_,
      std::vector<uint8_t>(9)));
  ASSERT_TRUE(Wait([&] {
        return receiver.Snapshot().truncated_ == 1;
      }));
  EXPECT_EQ(calls, 0);
  EXPECT_TRUE(test::SendUdp(receiver.Snapshot().port_, { 1 }));
  EXPECT_TRUE(Wait([&] {
        return calls == 1;
      }));
}

TEST(UdpReceiver, CallbackExceptionStopsThreadAndRetainsDiagnostic) {
  UdpReceiver receiver;
  ASSERT_TRUE(receiver.Start("127.0.0.1", 0, 8,
      [](const uint8_t*, size_t) {
        throw std::runtime_error("injected");
      }));
  ASSERT_TRUE(test::SendUdp(receiver.Snapshot().port_, { 1 }));
  ASSERT_TRUE(Wait([&] {
        return !receiver.Snapshot().running_;
      }));
  receiver.Stop();
  EXPECT_EQ(receiver.Snapshot().errors_, 1U);
  EXPECT_NE(receiver.Snapshot().last_error_.find("injected"),
      std::string::npos);
}

}  // namespace
}  // namespace astrall_driver
