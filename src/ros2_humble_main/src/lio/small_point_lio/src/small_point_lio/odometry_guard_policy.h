#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>

namespace small_point_lio {

    enum class OdometryDecision {
        Accept,
        Quarantine,
        Fault,
    };

    /**
     * Confirms kinematic odometry violations without turning a single scan
     * matching correction into a latched fault.
     *
     * Numerical/filter-health failures bypass confirmation and fault
     * immediately. Kinematic failures must repeat on consecutive candidates;
     * the exact reason may change while the state is deteriorating.
     */
    class OdometryGuardPolicy {
    public:
        explicit OdometryGuardPolicy(const int64_t required_samples = 2)
            : required_samples_(std::max<int64_t>(1, required_samples)) {}

        void set_required_samples(const int64_t required_samples) {
            required_samples_ = std::max<int64_t>(1, required_samples);
            reset();
        }

        [[nodiscard]] OdometryDecision observe(
            const std::string_view reason,
            const bool immediate_fault) {
            if (reason.empty()) {
                reset();
                return OdometryDecision::Accept;
            }

            if (immediate_fault) {
                consecutive_samples_ = 1;
                last_reason_ = reason;
                return OdometryDecision::Fault;
            }

            ++consecutive_samples_;
            last_reason_ = reason;

            if (consecutive_samples_ >= required_samples_) {
                return OdometryDecision::Fault;
            }
            return OdometryDecision::Quarantine;
        }

        void reset() {
            consecutive_samples_ = 0;
            last_reason_.clear();
        }

        [[nodiscard]] int64_t consecutive_samples() const {
            return consecutive_samples_;
        }

        [[nodiscard]] int64_t required_samples() const {
            return required_samples_;
        }

        [[nodiscard]] const std::string &last_reason() const {
            return last_reason_;
        }

    private:
        int64_t required_samples_{2};
        int64_t consecutive_samples_{0};
        std::string last_reason_;
    };

    /**
     * Treats short vertical corrections as a quadruped gait transient.
     *
     * A vertical-only violation is always quarantined.  It may become a
     * latched fault only after it has persisted for the configured duration
     * and the same sample also contains independent planar or yaw evidence.
     * This prevents two adjacent footfall/body-bob samples from stopping the
     * robot while retaining a bounded path for a genuinely diverging state.
     */
    class VerticalOdometryGuardPolicy {
    public:
        explicit VerticalOdometryGuardPolicy(
            const double required_duration_seconds = 2.0)
            : required_duration_seconds_(
                  std::max(0.1, required_duration_seconds)) {}

        void set_required_duration(const double required_duration_seconds) {
            required_duration_seconds_ =
                std::max(0.1, required_duration_seconds);
            reset();
        }

        [[nodiscard]] OdometryDecision observe(
            const double timestamp,
            const bool has_planar_or_yaw_corroboration) {
            if (!active_) {
                active_ = true;
                first_timestamp_ = timestamp;
                sample_count_ = 1;
            } else {
                ++sample_count_;
            }
            duration_seconds_ =
                std::max(0.0, timestamp - first_timestamp_);
            has_corroboration_ = has_planar_or_yaw_corroboration;
            if (duration_seconds_ >= required_duration_seconds_ &&
                has_corroboration_) {
                return OdometryDecision::Fault;
            }
            return OdometryDecision::Quarantine;
        }

        void reset() {
            active_ = false;
            sample_count_ = 0;
            first_timestamp_ = 0.0;
            duration_seconds_ = 0.0;
            has_corroboration_ = false;
        }

        [[nodiscard]] bool active() const { return active_; }
        [[nodiscard]] int64_t sample_count() const { return sample_count_; }
        [[nodiscard]] double duration_seconds() const {
            return duration_seconds_;
        }
        [[nodiscard]] double required_duration_seconds() const {
            return required_duration_seconds_;
        }
        [[nodiscard]] bool has_corroboration() const {
            return has_corroboration_;
        }

    private:
        double required_duration_seconds_{2.0};
        bool active_{false};
        int64_t sample_count_{0};
        double first_timestamp_{0.0};
        double duration_seconds_{0.0};
        bool has_corroboration_{false};
    };

    /**
     * A weak scan match is a measurement-quality failure, not proof that the
     * estimator state diverged. Keep rolling suspect scans back until a
     * healthy scan arrives; this policy deliberately has no Fault result.
     */
    class ScanQualityRecoveryPolicy {
    public:
        explicit ScanQualityRecoveryPolicy(
            const double diagnostic_window_seconds = 5.0)
            : duration_policy_(diagnostic_window_seconds) {}

        void set_diagnostic_window(const double seconds) {
            duration_policy_.set_required_duration(seconds);
        }

        [[nodiscard]] OdometryDecision observe(const double timestamp) {
            return duration_policy_.observe(timestamp, false);
        }

        void reset() { duration_policy_.reset(); }
        [[nodiscard]] bool active() const {
            return duration_policy_.active();
        }
        [[nodiscard]] int64_t sample_count() const {
            return duration_policy_.sample_count();
        }
        [[nodiscard]] double duration_seconds() const {
            return duration_policy_.duration_seconds();
        }
        [[nodiscard]] double required_duration_seconds() const {
            return duration_policy_.required_duration_seconds();
        }

    private:
        VerticalOdometryGuardPolicy duration_policy_;
    };

}// namespace small_point_lio
