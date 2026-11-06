// Copyright 2026 Hypertron ROS 2 contributors

#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "astrall_driver/mode_state_machine.hpp"
#include "astrall_driver/text_rate_limiter.hpp"
#include "astrall_driver/velocity_controller.hpp"
#include "gtest/gtest.h"

namespace astrall_driver {
namespace {

using namespace std::chrono_literals;
const auto kNow = std::chrono::steady_clock::time_point{};

TEST(VelocityController, DefaultsClampAndResendUntilSteadyWatchdogExpires) {
  VelocityController controller({ 0.5, 0.6, 0.7 }, 100ms);
  EXPECT_FALSE(controller.Sample(kNow, true).send_);
  ASSERT_TRUE(controller.Accept({ 2, -3, 4 }, kNow, true));
  auto sample = controller.Sample(kNow + 20ms, true);
  EXPECT_TRUE(sample.send_);
  EXPECT_DOUBLE_EQ(sample.command_.forward_, 0.5);
  EXPECT_DOUBLE_EQ(sample.command_.lateral_, -0.6);
  EXPECT_DOUBLE_EQ(sample.command_.yaw_, 0.7);
  EXPECT_TRUE(controller.Sample(kNow + 80ms, true).send_);
  sample = controller.Sample(kNow + 100ms, true);
  EXPECT_TRUE(sample.expired_);
  EXPECT_DOUBLE_EQ(sample.command_.forward_, 0);
  controller.MarkSent(sample.revision_);
  EXPECT_FALSE(controller.Sample(kNow + 120ms, true).send_);
  EXPECT_FALSE(controller.HasCommand());
}

TEST(VelocityController,
    InvalidInputOrLostGateCannotReplayAndOldZeroCannotClearNewZero) {
  VelocityController controller;
  ASSERT_TRUE(controller.Accept({ 0.4, 0, 0 }, kNow, true));
  auto old = controller.Sample(kNow, true);
  EXPECT_FALSE(controller.Accept({ std::numeric_limits<double>::quiet_NaN(), 0,
                                   0 },
      kNow + 1ms, true));
  controller.MarkSent(old.revision_);
  auto zero = controller.Sample(kNow + 2ms, true);
  EXPECT_TRUE(zero.send_);
  EXPECT_DOUBLE_EQ(zero.command_.forward_, 0);
  controller.MarkSent(zero.revision_);
  EXPECT_FALSE(controller.Sample(kNow + 3ms, true).send_);
  ASSERT_TRUE(controller.Accept({ 0.2, 0, 0 }, kNow + 4ms, true));
  EXPECT_DOUBLE_EQ(controller.Sample(kNow + 5ms, false).command_.forward_, 0);
  EXPECT_FALSE(controller.HasCommand());
  EXPECT_THROW(VelocityController({ 2, 1, 1 }), std::invalid_argument);
}

TEST(TextRateLimiter, ExactBoundaryAndNoBurstOrRoundingToFiveHz) {
  TextRateLimiter limiter;
  EXPECT_TRUE(limiter.TrySend(kNow));
  EXPECT_FALSE(limiter.TrySend(kNow + 249999999ns));
  EXPECT_TRUE(limiter.TrySend(kNow + 250ms));
  EXPECT_TRUE(limiter.TrySend(kNow + 10s));
  EXPECT_FALSE(limiter.TrySend(kNow + 10s));
  TextRateLimiter boundary(std::nextafter(5.0, 0.0));
  EXPECT_TRUE(boundary.TrySend(kNow));
  EXPECT_FALSE(boundary.TrySend(kNow + 200ms));
  EXPECT_TRUE(boundary.TrySend(kNow + 200000001ns));
  boundary.RecordCompleted(kNow + 210ms);
  EXPECT_FALSE(boundary.Ready(kNow + 410ms));
  boundary.Configure(4);
  EXPECT_FALSE(boundary.Ready(kNow + 459ms));
  EXPECT_THROW(TextRateLimiter(5), std::invalid_argument);
  EXPECT_THROW(TextRateLimiter(0), std::invalid_argument);
  EXPECT_THROW(TextRateLimiter(std::numeric_limits<double>::infinity()),
      std::invalid_argument);
}

TEST(ModeStateMachine,
    RulesOneAndEightDampingPreemptsAnyStateAndRejectsOldAck) {
  for (auto status :
      { ASTRALL_SPORT_STATUS_UNKNOWN, ASTRALL_SPORT_STATUS_CHARGING,
        ASTRALL_SPORT_STATUS_FIXEDSTANDING,
        ASTRALL_SPORT_STATUS_MOVE }) {
    ModeStateMachine machine;
    machine.Observe(status, kNow);
    EXPECT_TRUE(machine.Begin(ASTRALL_SPORT_MODE_CMD_DAMPING, false,
        kNow).accepted_);
  }
  ModeStateMachine machine;
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDDOWN, kNow);
  const auto old =
      machine.Begin(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, false, kNow);
  const auto damping =
      machine.Begin(ASTRALL_SPORT_MODE_CMD_DAMPING, false, kNow);
  EXPECT_FALSE(machine.Acknowledge(old.ticket_, ASTRALL_RES_SUCCESSED, kNow));
  EXPECT_TRUE(machine.Acknowledge(damping.ticket_, ASTRALL_RES_RUNNING, kNow));
  EXPECT_TRUE(machine.Pending());
  machine.Observe(ASTRALL_SPORT_STATUS_DAMPING, kNow + 10ms);
  EXPECT_FALSE(machine.Pending());
  EXPECT_EQ(machine.Result(), ASTRALL_RES_SUCCESSED);
}

TEST(ModeStateMachine,
    RuleTwoWaitsForActualCompletionNotAckAndRunningIsNotFailure) {
  ModeStateMachine machine;
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDDOWN, kNow);
  auto stand = machine.Begin(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, false, kNow);
  ASSERT_TRUE(stand.accepted_);
  EXPECT_TRUE(machine.Acknowledge(stand.ticket_, ASTRALL_RES_SUCCESSED, kNow));
  EXPECT_TRUE(machine.Pending());
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDSTANDING, kNow + 10ms);
  EXPECT_FALSE(machine.Begin(ASTRALL_SPORT_MODE_CMD_MOVE, false,
      kNow + 10ms).accepted_);
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDSTAND, kNow + 20ms);
  EXPECT_FALSE(machine.Pending());
  auto move = machine.Begin(ASTRALL_SPORT_MODE_CMD_MOVE, false, kNow + 20ms);
  ASSERT_TRUE(move.accepted_);
  EXPECT_TRUE(machine.Acknowledge(move.ticket_, ASTRALL_RES_RUNNING,
      kNow + 20ms));
  EXPECT_EQ(machine.Result(), ASTRALL_RES_RUNNING);
  EXPECT_FALSE(machine.CanMove(kNow + 21ms));
  machine.Observe(ASTRALL_SPORT_STATUS_MOVE, kNow + 30ms);
  EXPECT_TRUE(machine.CanMove(kNow + 30ms));
}

TEST(ModeStateMachine,
    RulesThreeAndTenRequireCompletedStandAndNoInventedXOpcode) {
  for (auto command :
      { ASTRALL_SPORT_MODE_CMD_MOVE, ASTRALL_SPORT_MODE_CMD_MOVE_TANK }) {
    ModeStateMachine machine;
    machine.Observe(ASTRALL_SPORT_STATUS_FIXEDSTANDING, kNow);
    EXPECT_FALSE(machine.Begin(command, false, kNow).accepted_);
    machine.Observe(ASTRALL_SPORT_STATUS_FIXEDDOWN, kNow);
    EXPECT_FALSE(machine.Begin(command, false, kNow).accepted_);
    machine.Observe(ASTRALL_SPORT_STATUS_FIXEDSTAND, kNow);
    EXPECT_TRUE(machine.Begin(command, false, kNow).accepted_);
  }
  ModeStateMachine machine;
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDSTAND, kNow);
  EXPECT_EQ(machine.Begin(ASTRALL_SPORT_STATUS_MOVE_X, false, kNow).result_,
      ASTRALL_RES_INVALID_PARAM);
}

TEST(ModeStateMachine, RulesFourAndFiveValidateUnsupportedChargeRequests) {
  ModeStateMachine machine;
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDDOWN, kNow);
  EXPECT_EQ(machine.Begin(ASTRALL_SPORT_MODE_CMD_AUTOCHARGE, false,
      kNow).result_,
      ASTRALL_RES_FAILED);
  EXPECT_EQ(machine.Begin(ASTRALL_SPORT_MODE_CMD_EXITCHARGE, false,
      kNow).result_,
      ASTRALL_RES_FAILED);
  for (auto status :
      { ASTRALL_SPORT_STATUS_FIXEDSTAND, ASTRALL_SPORT_STATUS_MOVE }) {
    machine.Observe(status, kNow);
    auto result = machine.Begin(ASTRALL_SPORT_MODE_CMD_AUTOCHARGE, false, kNow);
    EXPECT_FALSE(result.accepted_);
    EXPECT_EQ(result.result_, ASTRALL_RES_INVALID_PARAM);
  }
  for (auto status :
      { ASTRALL_SPORT_STATUS_ATCHARGING, ASTRALL_SPORT_STATUS_CHARGING }) {
    machine.Observe(status, kNow);
    EXPECT_EQ(machine.Begin(ASTRALL_SPORT_MODE_CMD_EXITCHARGE, false,
        kNow).result_,
        ASTRALL_RES_INVALID_PARAM);
  }
}

TEST(ModeStateMachine,
    RulesSixSevenAndNineRestrictDockAndRequireExplicitRebootException) {
  for (auto status :
      { ASTRALL_SPORT_STATUS_ATCHARGING, ASTRALL_SPORT_STATUS_CHARGING }) {
    ModeStateMachine machine;
    machine.Observe(status, kNow);
    for (auto command :
        { ASTRALL_SPORT_MODE_CMD_FIXEDDOWN,
          ASTRALL_SPORT_MODE_CMD_FIXEDSTAND,
          ASTRALL_SPORT_MODE_CMD_MOVE,
          ASTRALL_SPORT_MODE_CMD_MOVE_TANK }) {
      EXPECT_FALSE(machine.Begin(command, true, kNow).accepted_);
    }
  }
  ModeStateMachine enabled(3000ms, 250ms, true);
  enabled.Observe(ASTRALL_SPORT_STATUS_ATCHARGING, kNow);
  EXPECT_FALSE(enabled.Begin(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, false,
      kNow).accepted_);
  EXPECT_TRUE(enabled.Begin(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, true,
      kNow).accepted_);
  ModeStateMachine charging(3000ms, 250ms, true);
  charging.Observe(ASTRALL_SPORT_STATUS_CHARGING, kNow);
  EXPECT_FALSE(charging.Begin(ASTRALL_SPORT_MODE_CMD_FIXEDSTAND, true,
      kNow).accepted_);
}

TEST(ModeStateMachine, TimeoutStaleUnknownAndFailureRemainObservable) {
  ModeStateMachine machine(100ms, 50ms);
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDSTAND, kNow);
  EXPECT_FALSE(machine.Begin(ASTRALL_SPORT_MODE_CMD_MOVE, false,
      kNow + 51ms).accepted_);
  machine.Observe(static_cast<AstrallSportStatus>(0xBEEF), kNow);
  EXPECT_FALSE(machine.Begin(ASTRALL_SPORT_MODE_CMD_FIXEDDOWN, false,
      kNow).accepted_);
  machine.Observe(ASTRALL_SPORT_STATUS_FIXEDSTAND, kNow);
  auto request = machine.Begin(ASTRALL_SPORT_MODE_CMD_MOVE, false, kNow);
  machine.Acknowledge(request.ticket_, ASTRALL_RES_RUNNING, kNow);
  EXPECT_TRUE(machine.Tick(kNow + 100ms));
  EXPECT_EQ(machine.Result(), ASTRALL_RES_TIMEOUT);
  auto damping = machine.Begin(ASTRALL_SPORT_MODE_CMD_DAMPING,
          false,
          kNow + 101ms);
  machine.Acknowledge(damping.ticket_, ASTRALL_RES_WITHOUT_AUTH, kNow + 101ms);
  EXPECT_FALSE(machine.Pending());
  EXPECT_EQ(machine.Result(), ASTRALL_RES_WITHOUT_AUTH);
}

TEST(ModeStateMachine, LateMatchingObservationOrAckCannotBypassDeadline) {
  ModeStateMachine observation(100ms, 250ms);
  observation.Observe(ASTRALL_SPORT_STATUS_FIXEDSTAND, kNow);
  auto request = observation.Begin(ASTRALL_SPORT_MODE_CMD_MOVE, false, kNow);
  observation.Acknowledge(request.ticket_, ASTRALL_RES_RUNNING, kNow);
  observation.Observe(ASTRALL_SPORT_STATUS_MOVE, kNow + 100ms);
  EXPECT_EQ(observation.Result(), ASTRALL_RES_TIMEOUT);
  EXPECT_FALSE(observation.CanMove(kNow + 100ms));
  ModeStateMachine ack(100ms, 250ms);
  ack.Observe(ASTRALL_SPORT_STATUS_FIXEDSTAND, kNow);
  request = ack.Begin(ASTRALL_SPORT_MODE_CMD_MOVE, false, kNow);
  ack.Observe(ASTRALL_SPORT_STATUS_MOVE, kNow + 50ms);
  EXPECT_FALSE(ack.Acknowledge(request.ticket_, ASTRALL_RES_SUCCESSED,
      kNow + 101ms));
  EXPECT_EQ(ack.Result(), ASTRALL_RES_TIMEOUT);
}

}  // namespace
}  // namespace astrall_driver
