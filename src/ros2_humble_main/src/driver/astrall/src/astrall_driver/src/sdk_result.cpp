// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/sdk_result.hpp"

#include <iomanip>
#include <sstream>

#include "astrall_sdk/interface.h"

namespace astrall_driver {

SdkResult::SdkResult(uint16_t value) : value_(value) {
}

uint16_t SdkResult::Value() const {
  return value_;
}

bool SdkResult::Succeeded() const {
  return value_ == ASTRALL_RES_SUCCESSED;
}

const char* SdkResult::Name() const {
  switch (value_) {
    case ASTRALL_RES_FAILED: return "ASTRALL_RES_FAILED";
    case ASTRALL_RES_TIMEOUT: return "ASTRALL_RES_TIMEOUT";
    case ASTRALL_RES_RUNNING: return "ASTRALL_RES_RUNNING";
    case ASTRALL_RES_SUCCESSED: return "ASTRALL_RES_SUCCESSED";
    case ASTRALL_RES_INVALID_PARAM: return "ASTRALL_RES_INVALID_PARAM";
    case ASTRALL_RES_NOT_INIT: return "ASTRALL_RES_NOT_INIT";
    case ASTRALL_RES_RC_NORELEASE: return "ASTRALL_RES_RC_NORELEASE";
    case ASTRALL_RES_BEEN_OBTAINED: return "ASTRALL_RES_BEEN_OBTAINED";
    case ASTRALL_RES_WITHOUT_AUTH: return "ASTRALL_RES_WITHOUT_AUTH";
    default: return "UNKNOWN_SDK_RESULT";
  }
}

std::string SdkResult::ToString() const {
  constexpr int kHexWidth = 4;
  std::ostringstream stream;
  stream << Name() << " (0x" << std::uppercase << std::hex
         << std::setfill('0') << std::setw(kHexWidth) << value_ << ")";
  return stream.str();
}

}  // namespace astrall_driver
