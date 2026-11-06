// Copyright 2026 Hypertron ROS 2 contributors

#include <atomic>
#include <cstdint>
#include <exception>
#include <stdexcept>

#include "astrall_driver/fake_sdk_client.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

constexpr uint32_t kWaitMs = 1000;

struct ExceptionProbe {
  std::atomic<bool> armed_{ false };
  std::atomic<int> reentries_{ 0 };
  std::atomic<uint64_t> observed_errors_{ 0 };
};

/** @brief 诊断槽位释放最后一个异常引用时，模拟用户资源析构重入。 */
class RetainedError : public std::exception {
 public:
  RetainedError(FakeSdkClient* client, ExceptionProbe* probe)
    : client_(client), probe_(probe) {
  }

  ~RetainedError() noexcept override {
    if (probe_->armed_) {
      probe_->observed_errors_ =
          client_->GetCallbackStatistics().callback_errors_;
      ++probe_->reentries_;
    }
  }

 private:
  FakeSdkClient* client_;
  ExceptionProbe* probe_;
};

void VerifyLatestExceptionAndClose(FakeSdkClient* client) {
  auto statistics = client->GetCallbackStatistics();
  EXPECT_EQ(statistics.callback_errors_, 2U);
  ASSERT_NE(statistics.last_exception_, nullptr);
  try {
    std::rethrow_exception(statistics.last_exception_);
  } catch (const std::runtime_error& error) {
    EXPECT_STREQ(error.what(), "second callback failure");
  } catch (...) {
    FAIL() << "The latest callback exception was not retained";
  }
  EXPECT_TRUE(client->EmitHeartbeat());
  ASSERT_TRUE(client->WaitForCallbacks(kWaitMs));
  EXPECT_EQ(client->GetCallbackStatistics().callback_errors_, 2U);
  client->Deinit();
  EXPECT_TRUE(client->WaitForCallbacks(kWaitMs));
  EXPECT_EQ(client->Heartbeat().Value(), ASTRALL_RES_NOT_INIT);
}

void VerifyReplacement(bool reenter) {
  ExceptionProbe probe;
  FakeSdkClient client;
  std::atomic<int> calls{ 0 };
  SdkCallbacks callbacks;
  callbacks.heartbeat_ = [&client, &probe, &calls] {
        const auto call = ++calls;
        if (call == 1) {
          throw RetainedError(&client, &probe);
        }
        if (call == 2) {
          throw std::runtime_error("second callback failure");
        }
      };
  ASSERT_TRUE(client.Init(callbacks).Succeeded());
  EXPECT_TRUE(client.EmitHeartbeat());
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  // 不保存统计快照，确保第一条异常只由诊断槽位持有最后一个引用。
  EXPECT_EQ(client.GetCallbackStatistics().callback_errors_, 1U);
  probe.armed_ = reenter;
  EXPECT_TRUE(client.EmitHeartbeat());
  ASSERT_TRUE(client.WaitForCallbacks(kWaitMs));
  probe.armed_ = false;
  EXPECT_EQ(probe.reentries_, reenter ? 1 : 0);
  EXPECT_EQ(probe.observed_errors_, reenter ? 2U : 0U);
  VerifyLatestExceptionAndClose(&client);
  EXPECT_EQ(calls, 3);
}

TEST(SdkExceptionReentry, ReplacementWithoutDestructorReentry) {
  VerifyReplacement(false);
}

TEST(SdkExceptionReentry, ReplacementWithDestructorReentry) {
  VerifyReplacement(true);
}

}  // namespace
}  // namespace astrall_driver
