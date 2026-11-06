// Copyright 2026 Hypertron ROS 2 contributors

#ifndef ASTRALL_DRIVER__FAULT_BITS_HPP_
#define ASTRALL_DRIVER__FAULT_BITS_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace astrall_driver {

/** @brief 手册 5.7 / 实际 C++ SDK 的全部 error 位；未知位保留十六进制。 */
std::vector<std::string> DecodeErrorBits(uint32_t bits);
/** @brief 实际头文件为准：相机缺失属 error bit9，warn bit8 仍为未知。 */
std::vector<std::string> DecodeWarnBits(uint32_t bits);

}  // namespace astrall_driver

#endif  // ASTRALL_DRIVER__FAULT_BITS_HPP_
