// Copyright 2026 Hypertron ROS 2 contributors

#ifndef UDP_TEST_SENDER_HPP_
#define UDP_TEST_SENDER_HPP_

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <vector>

namespace astrall_driver {
namespace test {

// 测试流量固定loopback，显式检查socket/sendto/close，避免触及真实设备。
inline bool SendUdp(uint16_t port, const std::vector<uint8_t>& data) {
  const auto descriptor = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
  if (descriptor < 0) {
    return false;
  }
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  const auto sent = sendto(descriptor, data.data(), data.size(), 0,
          reinterpret_cast<sockaddr*>(&address), sizeof(address));
  const bool closed = close(descriptor) == 0;
  return sent == static_cast<ssize_t>(data.size()) && closed;
}

}  // namespace test
}  // namespace astrall_driver

#endif  // UDP_TEST_SENDER_HPP_
