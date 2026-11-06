// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/subscription_frequency.hpp"

namespace astrall_driver {

std::optional<AstrallSubscribeFreq> SubscriptionFrequency(int64_t hz) {
  switch (hz) {
    case 0: return ASTRALL_SUB_FREQ_CLOSE;
    case 1: return ASTRALL_SUB_FREQ_1HZ;
    case 25: return ASTRALL_SUB_FREQ_25HZ;
    case 50: return ASTRALL_SUB_FREQ_50HZ;
    case 125: return ASTRALL_SUB_FREQ_125HZ;
    case 250: return ASTRALL_SUB_FREQ_250HZ;
    default: return std::nullopt;
  }
}

}  // namespace astrall_driver
