#include "small_point_lio/odometry_guard_policy.h"
#include "small_point_lio/measurement_quality_policy.h"

#include <gtest/gtest.h>

namespace small_point_lio {
namespace {

    TEST(OdometryGuardPolicy, IsolatedQuadrupedHeightCorrectionRecovers) {
        OdometryGuardPolicy policy(2);
        constexpr double recorded_pose_vertical_speed = 1.091;
        constexpr double recorded_estimator_vertical_speed = 0.258;
        constexpr double recorded_vertical_step = 0.109;

        ASSERT_GT(recorded_pose_vertical_speed, 0.8);
        ASSERT_LT(recorded_estimator_vertical_speed, 0.8);
        ASSERT_LT(recorded_vertical_step, 0.20);

        EXPECT_EQ(
            policy.observe("pose_vertical_speed", false),
            OdometryDecision::Quarantine);
        EXPECT_EQ(policy.consecutive_samples(), 1);

        EXPECT_EQ(policy.observe("", false), OdometryDecision::Accept);
        EXPECT_EQ(policy.consecutive_samples(), 0);
        EXPECT_TRUE(policy.last_reason().empty());
    }

    TEST(OdometryGuardPolicy, ConsecutiveKinematicViolationsLatchFault) {
        OdometryGuardPolicy policy(2);

        EXPECT_EQ(
            policy.observe("pose_vertical_speed", false),
            OdometryDecision::Quarantine);
        // A deteriorating estimate can cross a different limit on the next
        // sample; it is still a consecutive kinematic violation.
        EXPECT_EQ(
            policy.observe("vertical_step", false),
            OdometryDecision::Fault);
        EXPECT_EQ(policy.consecutive_samples(), 2);
        EXPECT_EQ(policy.last_reason(), "vertical_step");
    }

    TEST(OdometryGuardPolicy, NumericalFailureFaultsImmediately) {
        OdometryGuardPolicy policy(3);

        EXPECT_EQ(
            policy.observe("pose_vertical_speed", false),
            OdometryDecision::Quarantine);
        EXPECT_EQ(
            policy.observe("non_finite_state", true),
            OdometryDecision::Fault);
        EXPECT_EQ(policy.consecutive_samples(), 1);
    }

    TEST(OdometryGuardPolicy, RequiredSamplesAreNeverBelowOne) {
        OdometryGuardPolicy policy(0);

        EXPECT_EQ(policy.required_samples(), 1);
        EXPECT_EQ(
            policy.observe("pose_linear_speed", false),
            OdometryDecision::Fault);
    }

    TEST(OdometryGuardPolicy, RecoveryWindowAllowsSevenRollbackAttempts) {
        OdometryGuardPolicy policy(8);
        for (int sample = 1; sample < 8; ++sample) {
            EXPECT_EQ(
                policy.observe("scan_matching_quality", false),
                OdometryDecision::Quarantine);
            EXPECT_EQ(policy.consecutive_samples(), sample);
        }
        EXPECT_EQ(
            policy.observe("scan_matching_quality", false),
            OdometryDecision::Fault);

        policy.reset();
        EXPECT_EQ(
            policy.observe("pose_linear_speed", false),
            OdometryDecision::Quarantine);
        EXPECT_EQ(policy.observe("", false), OdometryDecision::Accept);
    }

    TEST(MeasurementQualityPolicy, RecordedMultiAxisFootfallIsRejected) {
        // This sample was recorded immediately before the reported LIO
        // divergence. No individual axis exceeds 3 g, but the combined
        // impact is approximately 3.35 g.
        EXPECT_TRUE(is_acceleration_norm_outlier(
            -2.396039, 1.332417, 1.921297, 2.5));
        EXPECT_FALSE(is_acceleration_norm_outlier(
            -0.50, 0.10, 0.90, 2.5));
    }

    TEST(MeasurementQualityPolicy, ScanNeedsEnoughValidConstraints) {
        EXPECT_TRUE(is_scan_matching_healthy(
            1000, 100, 200, 40, 120, 0.05));
        EXPECT_FALSE(is_scan_matching_healthy(
            1000, 30, 200, 40, 120, 0.05));
        EXPECT_FALSE(is_scan_matching_healthy(
            100, 80, 200, 40, 120, 0.05));
        EXPECT_FALSE(is_scan_matching_healthy(
            1000, 40, 200, 40, 120, 0.05));
        EXPECT_TRUE(is_scan_matching_healthy(
            1000, 50, 200, 40, 120, 0.05));
        EXPECT_FALSE(is_scan_matching_healthy(
            1000, 50, 200, 40, 0, 0.05));
    }

    TEST(MeasurementQualityPolicy, RecordedMid360sConstraintsRemainUsable) {
        // The navigation bag that previously triggered a false hard stop had
        // thousands of attempted constraints and 191--223 accepted matches.
        // Its ratio was narrowly below the old 5% threshold, but the absolute
        // number of valid constraints remained far above the 40-point guard.
        EXPECT_TRUE(is_scan_matching_healthy(
            4299, 191, 200, 40, 120, 0.04));
        EXPECT_TRUE(is_scan_matching_healthy(
            4348, 207, 200, 40, 120, 0.04));
        EXPECT_TRUE(is_scan_matching_healthy(
            4666, 164, 200, 40, 120, 0.04));
        EXPECT_FALSE(is_scan_matching_healthy(
            4518, 40, 200, 40, 120, 0.04));
    }

    TEST(
        VerticalOdometryGuardPolicy,
        RecordedQuadrupedHeightBurstNeverLatchesFault) {
        VerticalOdometryGuardPolicy policy(2.0);

        EXPECT_EQ(
            policy.observe(100.000, false),
            OdometryDecision::Quarantine);
        EXPECT_EQ(
            policy.observe(100.223, false),
            OdometryDecision::Quarantine);
        EXPECT_EQ(
            policy.observe(100.303, false),
            OdometryDecision::Quarantine);
        EXPECT_EQ(policy.sample_count(), 3);
        EXPECT_NEAR(policy.duration_seconds(), 0.303, 1.0e-9);
        EXPECT_FALSE(policy.has_corroboration());

        policy.reset();
        EXPECT_FALSE(policy.active());
        EXPECT_EQ(policy.sample_count(), 0);
    }

    TEST(
        VerticalOdometryGuardPolicy,
        PersistentVerticalOnlyEvidenceRemainsQuarantined) {
        VerticalOdometryGuardPolicy policy(2.0);

        EXPECT_EQ(
            policy.observe(10.0, false),
            OdometryDecision::Quarantine);
        EXPECT_EQ(
            policy.observe(12.5, false),
            OdometryDecision::Quarantine);
        EXPECT_GT(policy.duration_seconds(), 2.0);
    }

    TEST(
        VerticalOdometryGuardPolicy,
        PersistentVerticalAndPlanarEvidenceLatchesFault) {
        VerticalOdometryGuardPolicy policy(2.0);

        EXPECT_EQ(
            policy.observe(10.0, true),
            OdometryDecision::Quarantine);
        EXPECT_EQ(
            policy.observe(11.9, true),
            OdometryDecision::Quarantine);
        EXPECT_EQ(
            policy.observe(12.0, true),
            OdometryDecision::Fault);
        EXPECT_TRUE(policy.has_corroboration());
    }

    TEST(
        ScanQualityRecoveryPolicy,
        RecordedMid360sScanQualityBurstAutomaticallyRecovers) {
        // The reported navigation failure contained a sub-second burst of
        // weak scan-quality samples. Scan quality has its own time-based
        // recovery window and cannot latch a permanent fault even when weak
        // matching lasts longer than the diagnostic window.
        ScanQualityRecoveryPolicy policy(5.0);
        for (int sample = 0; sample < 100; ++sample) {
            EXPECT_EQ(
                policy.observe(100.0 + sample * 0.1),
                OdometryDecision::Quarantine);
        }
        EXPECT_GT(policy.duration_seconds(), 5.0);

        policy.reset();
        EXPECT_FALSE(policy.active());
    }

}// namespace
}// namespace small_point_lio
