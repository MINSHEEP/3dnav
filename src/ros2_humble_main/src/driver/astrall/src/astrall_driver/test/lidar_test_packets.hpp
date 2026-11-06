// Copyright 2026 Hypertron ROS 2 contributors

#ifndef LIDAR_TEST_PACKETS_HPP_
#define LIDAR_TEST_PACKETS_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace astrall_driver {
namespace test {

// 独立按文档偏移造字节，不复制SDK或parser结构，兼顾非对齐和负数。
inline void Put(std::vector<uint8_t>* bytes, size_t at, uint64_t value,
    size_t width) {
  for (size_t i = 0; i < width; ++i) {
    bytes->at(at + i) = static_cast<uint8_t>(value >> (8 * i));
  }
}

inline std::vector<uint8_t> PointBytes(uint64_t stamp, uint32_t total,
    uint32_t index, uint16_t count, bool packed = false) {
  std::vector<uint8_t> bytes(packed ? 1422 : 1432);
  const size_t time = packed ? 2 : 8;
  const size_t start = packed ? 20 : 28;
  Put(&bytes, 0, 0xAA55, 2);
  Put(&bytes, time, stamp, 8);
  Put(&bytes, time + 8, total, 4);
  Put(&bytes, time + 12, index, 4);
  Put(&bytes, time + 16, count, 2);
  for (size_t i = 0; i < 50; ++i) {
    Put(&bytes, start + i * 28, 10000 + index + i, 4);
    Put(&bytes, start + i * 28 + 4, static_cast<uint32_t>(-20000), 4);
    Put(&bytes, start + i * 28 + 8, 5000, 4);
    Put(&bytes, start + i * 28 + 12, 10, 4);
    Put(&bytes, start + i * 28 + 16, 20, 4);
    Put(&bytes, start + i * 28 + 20, 30, 4);
    Put(&bytes, start + i * 28 + 24, 255, 4);
  }
  Put(&bytes, packed ? 1420 : 1428, 0xFF00, 2);
  return bytes;
}

inline std::vector<uint8_t> OdometryBytes(uint64_t stamp,
    bool packed = false) {
  std::vector<uint8_t> bytes(packed ? 68 : 80);
  const size_t time = packed ? 2 : 8;
  Put(&bytes, 0, 0xAA55, 2);
  Put(&bytes, time, stamp, 8);
  Put(&bytes, time + 8, 1, 8);
  Put(&bytes, time + 16, static_cast<uint64_t>(-2), 8);
  Put(&bytes, time + 24, 3, 8);
  Put(&bytes, time + 56, 2, 8);
  Put(&bytes, packed ? 66 : 72, 0xFF00, 2);
  return bytes;
}

}  // namespace test
}  // namespace astrall_driver

#endif  // LIDAR_TEST_PACKETS_HPP_
