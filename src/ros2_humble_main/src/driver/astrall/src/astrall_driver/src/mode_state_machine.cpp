// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/mode_state_machine.hpp"

#include <stdexcept>

namespace astrall_driver {
namespace {

bool Charging(AstrallSportStatus actual) {
  return actual == ASTRALL_SPORT_STATUS_ATCHARGING ||
         actual == ASTRALL_SPORT_STATUS_CHARGING;
}

bool Transitional(AstrallSportStatus actual) {
  return actual == ASTRALL_SPORT_STATUS_INIT ||
         actual == ASTRALL_SPORT_STATUS_FIXEDSTANDING ||
         actual == ASTRALL_SPORT_STATUS_FIXEDDOWNING ||
         actual == ASTRALL_SPORT_STATUS_AUTOCHAEGE ||
         actual == ASTRALL_SPORT_STATUS_SEARCHCHARGING ||
         actual == ASTRALL_SPORT_STATUS_PLUGGINGCHARGING ||
         actual == ASTRALL_SPORT_STATUS_EXITCHARGING;
}

bool KnownMode(AstrallSportStatus actual) {
  switch (actual) {
    case ASTRALL_SPORT_STATUS_DAMPING:
    case ASTRALL_SPORT_STATUS_FIXEDSTAND:
    case ASTRALL_SPORT_STATUS_FIXEDDOWN:
    case ASTRALL_SPORT_STATUS_MOVE:
    case ASTRALL_SPORT_STATUS_UPSTAIRS:
    case ASTRALL_SPORT_STATUS_CAR:
    case ASTRALL_SPORT_STATUS_ATCHARGING:
    case ASTRALL_SPORT_STATUS_CHARGING:
    case ASTRALL_SPORT_STATUS_MOVE_WEIGHT:
    case ASTRALL_SPORT_STATUS_MOVE_X:
    case ASTRALL_SPORT_STATUS_MOVE_TANK: return true;
    default: return Transitional(actual);
  }
}

uint16_t Target(uint16_t command) {
  switch (command) {
    case ASTRALL_SPORT_MODE_CMD_DAMPING: {
      return ASTRALL_SPORT_STATUS_DAMPING;
    }
    case ASTRALL_SPORT_MODE_CMD_FIXEDSTAND: {
      return ASTRALL_SPORT_STATUS_FIXEDSTAND;
    }
    case ASTRALL_SPORT_MODE_CMD_FIXEDDOWN: {
      return ASTRALL_SPORT_STATUS_FIXEDDOWN;
    }
    case ASTRALL_SPORT_MODE_CMD_MOVE: {
      return ASTRALL_SPORT_STATUS_MOVE;
    }
    case ASTRALL_SPORT_MODE_CMD_MOVE_TANK: {
      return ASTRALL_SPORT_STATUS_MOVE_TANK;
    }
    default: return ASTRALL_SPORT_STATUS_UNKNOWN;
  }
}

}  // namespace

ModeStateMachine::ModeStateMachine(std::chrono::milliseconds timeout,
    std::chrono::milliseconds freshness, bool allow_reboot_stand)
  : timeout_(timeout), freshness_(freshness),
  allow_reboot_stand_(allow_reboot_stand) {
  if (timeout.count() <= 0 || freshness.count() <= 0) {
    throw std::invalid_argument("Mode timeout/freshness must be positive");
  }
}

void ModeStateMachine::Observe(AstrallSportStatus actual, Time now) {
  Tick(now);
  actual_ = actual;
  observed_at_ = now;
  ++observations_;
  CompleteIfObserved();
}

ModeDecision ModeStateMachine::Check(uint16_t command, bool confirmation,
    Time now) const {
  if (command == ASTRALL_SPORT_MODE_CMD_DAMPING) {
    return { true, ASTRALL_RES_RUNNING, 0,
             "5.12.3 rules 1/8: damping priority" };
  }
  if (Target(command) == ASTRALL_SPORT_STATUS_UNKNOWN &&
      command != ASTRALL_SPORT_MODE_CMD_AUTOCHARGE &&
      command != ASTRALL_SPORT_MODE_CMD_EXITCHARGE) {
    return { false, ASTRALL_RES_INVALID_PARAM, 0,
             "SDK has no supported opcode" };
  }
  if (pending_ || Transitional(actual_)) {
    return { false, ASTRALL_RES_RUNNING, 0,
             "5.12.3 rule 2: previous mode incomplete" };
  }
  if (observations_ == 0 || !KnownMode(actual_) ||
      now - observed_at_ > freshness_) {
    return { false, ASTRALL_RES_FAILED, 0, "actual mode is unknown or stale" };
  }
  if (command == ASTRALL_SPORT_MODE_CMD_EXITCHARGE) {
    return { false,
             static_cast<uint16_t>(Charging(actual_) ? ASTRALL_RES_INVALID_PARAM
            : ASTRALL_RES_FAILED),
             0,
             Charging(actual_) ? "rule 5 valid; exit-charge product unsupported"
            :
             "5.12.3 rule 5: exit requires docked/charging" };
  }
  const bool reboot_exception = allow_reboot_stand_ && confirmation &&
      actual_ == ASTRALL_SPORT_STATUS_ATCHARGING &&
      command == ASTRALL_SPORT_MODE_CMD_FIXEDSTAND;
  if (Charging(actual_) && !reboot_exception) {
    return { false, ASTRALL_RES_FAILED, 0,
             "5.12.3 rules 6/7/9: dock restrictions" };
  }
  if ((command == ASTRALL_SPORT_MODE_CMD_MOVE ||
      command == ASTRALL_SPORT_MODE_CMD_MOVE_TANK) &&
      actual_ != ASTRALL_SPORT_STATUS_FIXEDSTAND) {
    return { false, ASTRALL_RES_FAILED, 0,
             "5.12.3 rules 3/10: completed stand required" };
  }
  if (command == ASTRALL_SPORT_MODE_CMD_AUTOCHARGE) {
    const bool valid = actual_ == ASTRALL_SPORT_STATUS_FIXEDSTAND ||
        actual_ == ASTRALL_SPORT_STATUS_MOVE;
    return { false,
             static_cast<uint16_t>(valid ? ASTRALL_RES_INVALID_PARAM :
             ASTRALL_RES_FAILED),
             0,
             valid ? "rule 4 valid; auto-charge product unsupported" :
             "5.12.3 rule 4: auto-charge requires stand/Move" };
  }
  return { true, ASTRALL_RES_RUNNING, 0, reboot_exception ?
           "rule 9: explicitly confirmed reboot on dock" :
           "mode preconditions valid" };
}

ModeDecision ModeStateMachine::Begin(uint16_t command, bool confirmation,
    Time now) {
  Tick(now);
  auto decision = Check(command, confirmation, now);
  if (!decision.accepted_) {
    return decision;
  }
  decision.ticket_ = ++ticket_;
  target_ = Target(command);
  request_observation_ = observations_;
  acknowledged_ = false;
  pending_ = true;
  failed_ = false;
  result_ = ASTRALL_RES_RUNNING;
  deadline_ = now + timeout_;
  reason_ = decision.reason_ + "; waiting for ACK and actual completed mode";
  return decision;
}

bool ModeStateMachine::Acknowledge(uint64_t ticket, uint16_t result, Time now) {
  if (!pending_ || ticket != ticket_) {
    return false;
  }
  if (Tick(now)) {
    return false;
  }
  if (result == ASTRALL_RES_SUCCESSED || result == ASTRALL_RES_RUNNING) {
    acknowledged_ = true;
    CompleteIfObserved();
  } else {
    result_ = result;
    pending_ = false;
    failed_ = true;
    reason_ = "SDK rejected mode request";
  }
  return true;
}

void ModeStateMachine::CompleteIfObserved() {
  if (pending_ && acknowledged_ && observations_ > request_observation_ &&
      static_cast<uint16_t>(actual_) == target_) {
    pending_ = false;
    result_ = ASTRALL_RES_SUCCESSED;
    reason_ = "actual completed mode observed";
  }
}

bool ModeStateMachine::Tick(Time now) {
  if (pending_ && now >= deadline_) {
    pending_ = false;
    result_ = ASTRALL_RES_TIMEOUT;
    failed_ = true;
    reason_ = "mode transition timeout";
    return true;
  }
  return false;
}

bool ModeStateMachine::CanMove(Time now) const {
  return !pending_ && !failed_ && observations_ != 0 &&
         now - observed_at_ <= freshness_ &&
         (actual_ == ASTRALL_SPORT_STATUS_MOVE ||
         actual_ == ASTRALL_SPORT_STATUS_MOVE_TANK);
}

void ModeStateMachine::Cancel(uint16_t result, const std::string& reason) {
  ++ticket_;
  pending_ = false;
  failed_ = true;
  result_ = result;
  reason_ = reason;
}

bool ModeStateMachine::Pending() const {
  return pending_;
}
AstrallSportStatus ModeStateMachine::Actual() const {
  return actual_;
}
uint16_t ModeStateMachine::Result() const {
  return result_;
}
uint64_t ModeStateMachine::Ticket() const {
  return ticket_;
}
uint16_t ModeStateMachine::TargetStatus() const {
  return target_;
}

const std::string& ModeStateMachine::Reason() const {
  return reason_;
}

}  // namespace astrall_driver
