// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__SUBSCRIPTION_FREQUENCY_HPP_
#define ASTRALL_DRIVER__SUBSCRIPTION_FREQUENCY_HPP_

#include <cstdint>
#include <optional>

#include "astrall_sdk/interface.h"

namespace astrall_driver {

/** @brief 手册 5.10：Hz 数值转换为枚举序号；不能把 250 直接强转为枚举。 */
std::optional<AstrallSubscribeFreq> SubscriptionFrequency(int64_t hz);

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__SUBSCRIPTION_FREQUENCY_HPP_
