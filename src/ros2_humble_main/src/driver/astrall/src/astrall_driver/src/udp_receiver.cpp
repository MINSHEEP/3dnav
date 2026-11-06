// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/udp_receiver.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <exception>
#include <system_error>
#include <vector>

namespace astrall_driver {
namespace {

constexpr size_t kMaxUdpPayload = 65507;
constexpr int kStopPollMs = 100;

std::string SystemError(const std::string& operation) {
  return operation + ": " + std::error_code(errno,
             std::generic_category()).message();
}

}  // namespace

UdpReceiver::~UdpReceiver() {
  Stop();
}

void UdpReceiver::Error(const std::string& message) {
  std::lock_guard<std::mutex> lock(mutex_);
  ++snapshot_.errors_;
  snapshot_.last_error_ = message;
}

UdpReceiverSnapshot UdpReceiver::Snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto result = snapshot_;
  result.running_ = running_;
  return result;
}

void UdpReceiver::CloseDescriptors() {
  for (int* descriptor : { &socket_, &wake_read_, &wake_write_ }) {
    if (*descriptor >= 0 && close(*descriptor) != 0) {
      Error(SystemError("close UDP descriptor"));
    }
    *descriptor = -1;
  }
}

bool UdpReceiver::Start(const std::string& address, uint16_t port,
    size_t max_size, Callback callback) {
  if (thread_.joinable()) {
    Error("UDP receiver must Stop before Start");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = {};
  }
  sockaddr_in local{};
  local.sin_family = AF_INET;
  local.sin_port = htons(port);
  if (!callback || max_size == 0 || max_size > kMaxUdpPayload ||
      inet_pton(AF_INET, address.c_str(), &local.sin_addr) != 1) {
    Error("invalid UDP IPv4 address/callback/buffer size");
    return false;
  }
  socket_ = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
  if (socket_ < 0 || bind(socket_, reinterpret_cast<sockaddr*>(&local),
      sizeof(local)) != 0) {
    Error(SystemError("UDP socket/bind"));
    CloseDescriptors();
    return false;
  }
  socklen_t length = sizeof(local);
  int wake[2];
  if (getsockname(socket_, reinterpret_cast<sockaddr*>(&local), &length) != 0 ||
      pipe2(wake, O_NONBLOCK | O_CLOEXEC) != 0) {
    Error(SystemError("UDP getsockname/pipe2"));
    CloseDescriptors();
    return false;
  }
  wake_read_ = wake[0];
  wake_write_ = wake[1];
  {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_.port_ = ntohs(local.sin_port);
  }
  stop_ = false;
  running_ = true;
  try {
    thread_ = std::thread([this, max_size, callback] {
        Run(max_size, callback);
      });
  } catch (const std::exception& error) {
    Error(std::string("UDP thread start: ") + error.what());
    stop_ = true;
    running_ = false;
    CloseDescriptors();
    return false;
  }
  return true;
}

void UdpReceiver::Stop() {
  stop_ = true;
  if (thread_.joinable()) {
    const uint8_t wake = 1;
    if (write(wake_write_, &wake, 1) < 0 && errno != EAGAIN && errno != EINTR) {
      Error(SystemError("UDP stop wake"));
    }
    thread_.join();
  }
  running_ = false;
  CloseDescriptors();
}

void UdpReceiver::Run(size_t max_size, const Callback& callback) {
  try {
    std::vector<uint8_t> data(max_size);
    pollfd descriptors[2] = { { socket_, POLLIN, 0 },
      { wake_read_, POLLIN, 0 } };
    while (!stop_) {
      const auto ready = poll(descriptors, 2, kStopPollMs);
      if (stop_ || (descriptors[1].revents & POLLIN)) {
        break;
      }
      if (ready < 0 && errno == EINTR) {
        continue;
      }
      if (ready < 0 || (descriptors[0].revents &
          (POLLERR | POLLHUP | POLLNVAL))) {
        Error(ready < 0 ? SystemError("UDP poll") : "UDP poll socket error");
        break;
      }
      if (!(descriptors[0].revents & POLLIN)) {
        continue;
      }
      iovec vector{ data.data(), data.size() };
      msghdr message{};
      message.msg_iov = &vector;
      message.msg_iovlen = 1;
      const auto size = recvmsg(socket_, &message, 0);
      if (size < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
          continue;
        }
        Error(SystemError("UDP recvmsg"));
        break;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.datagrams_;
        if (message.msg_flags & MSG_TRUNC) {
          ++snapshot_.truncated_;
          continue;
        }
      }
      callback(data.data(), static_cast<size_t>(size));
    }
  } catch (const std::exception& error) {
    Error(std::string("UDP receive/callback exception: ") + error.what());
  } catch (...) {
    Error("UDP receive/callback unknown exception");
  }
  running_ = false;
}

}  // namespace astrall_driver
