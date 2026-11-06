// Copyright 2026 Hypertron ROS 2 contributors

#include "astrall_driver/text_rate_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astrall_driver {

TextRateLimiter::TextRateLimiter(double rate) {
  Configure(rate);
}

void TextRateLimiter::Configure(double rate) {
  constexpr double kMinimumRate = 0.1;
  constexpr double kMaximumRate = 5;
  if (!std::isfinite(rate) || rate < kMinimumRate || rate >= kMaximumRate) {
    throw std::invalid_argument("Text rate must be finite and in [0.1,5) Hz");
  }
  const auto requested = std::chrono::ceil<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(1.0 / rate));
  const auto strict_minimum = std::chrono::milliseconds(200) +
      std::chrono::steady_clock::duration{ 1 };
  interval_ = std::max(requested, strict_minimum);
}

bool TextRateLimiter::Ready(Time now) const {
  return !last_ || now - *last_ >= interval_;
}

bool TextRateLimiter::TrySend(Time now) {
  if (!Ready(now)) {
    return false;
  }
  last_ = now;
  return true;
}

void TextRateLimiter::RecordCompleted(Time now) {
  if (!last_ || now > *last_) {
    last_ = now;
  }
}

}  // namespace astrall_driver
