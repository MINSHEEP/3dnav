#pragma once

#include <cmath>
#include <cstddef>

namespace small_point_lio {

    [[nodiscard]] inline bool is_acceleration_norm_outlier(
        const double x,
        const double y,
        const double z,
        const double maximum_norm) {
        if (!std::isfinite(x) || !std::isfinite(y) ||
            !std::isfinite(z) || !std::isfinite(maximum_norm) ||
            maximum_norm <= 0.0) {
            return true;
        }
        return std::hypot(x, y, z) >= maximum_norm;
    }

    [[nodiscard]] inline bool is_scan_matching_healthy(
        const std::size_t attempted,
        const std::size_t accepted,
        const std::size_t minimum_attempted,
        const std::size_t minimum_accepted,
        const std::size_t strong_absolute_accepted,
        const double minimum_acceptance_ratio) {
        if (attempted == 0 || attempted < minimum_attempted ||
            accepted < minimum_accepted ||
            strong_absolute_accepted < minimum_accepted ||
            !std::isfinite(minimum_acceptance_ratio) ||
            minimum_acceptance_ratio <= 0.0 ||
            minimum_acceptance_ratio > 1.0) {
            return false;
        }
        const double ratio =
            static_cast<double>(accepted) / static_cast<double>(attempted);
        return ratio >= minimum_acceptance_ratio ||
               accepted >= strong_absolute_accepted;
    }

}// namespace small_point_lio
