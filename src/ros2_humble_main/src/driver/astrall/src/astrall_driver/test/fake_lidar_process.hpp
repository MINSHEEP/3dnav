// Copyright 2026 Hypertron ROS 2 contributors

#ifndef FAKE_LIDAR_PROCESS_HPP_
#define FAKE_LIDAR_PROCESS_HPP_

#include <spawn.h>
#include <sys/wait.h>

#include <cerrno>
#include <cstdint>
#include <string>
#include <vector>

extern char** environ;

namespace astrall_driver {
namespace test {

// 无shell的独立Python进程，固定loopback；外层CTest设进程超时。
inline int RunFakeLidar(uint16_t cloud_port, uint16_t odom_port,
    uint64_t stamp, const std::string& scenario, bool packed = false) {
  std::vector<std::string> values{ "/usr/bin/python3", FAKE_LIDAR_SENDER_PATH,
                                   "--point-port", std::to_string(cloud_port),
                                   "--odom-port",
                                   std::to_string(odom_port), "--timestamp-ns",
                                   std::to_string(stamp),
                                   "--points", "101", "--scenario", scenario,
                                   "--wire-format",
                                   packed ? "packed_le" : "native_le" };
  std::vector<char*> arguments;
  for (auto& value : values) {
    arguments.push_back(value.data());
  }
  arguments.push_back(nullptr);
  pid_t process;
  const auto launched = posix_spawn(&process, values[0].c_str(), nullptr,
          nullptr, arguments.data(), environ);
  if (launched != 0) {
    return launched;
  }
  int result = 0;
  pid_t waited;
  do {
    waited = waitpid(process, &result, 0);
  } while (waited < 0 && errno == EINTR);
  return waited == process && WIFEXITED(result) ? WEXITSTATUS(result) : -1;
}

}  // namespace test
}  // namespace astrall_driver

#endif  // FAKE_LIDAR_PROCESS_HPP_
