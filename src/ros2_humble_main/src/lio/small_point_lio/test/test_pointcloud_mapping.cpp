#include <gtest/gtest.h>

#include <Eigen/Core>
#include <vector>

#include "util/pointcloud_mapping.h"

namespace {

util::PointcloudMapping make_mapping() {
    return util::PointcloudMapping(
        0.10F,  // voxel size
        3,      // observations
        1,      // stride
        2,      // maximum observation gap
        2,      // minimum stable span
        20,     // stale frames
        10,     // prune interval
        1000,   // capacity
        util::PointcloudMappingGroundPolicy{});
}

util::PointcloudMapping make_ground_mapping() {
    return util::PointcloudMapping(
        0.10F, 6, 1, 2, 5, 20, 10, 1000,
        util::PointcloudMappingGroundPolicy{
            true, 0.20F, 0.08F, 30.0F, 3, 3, 2});
}

util::PointcloudMapping make_filter_mode_mapping(
        const util::PointcloudMappingFilterMode mode) {
    return util::PointcloudMapping(
        0.10F, 6, 1, 2, 5, 20, 10, 1000,
        util::PointcloudMappingGroundPolicy{
            true, 0.20F, 0.08F, 30.0F, 3, 3, 2, mode});
}

std::vector<Eigen::Vector3f> flat_ground_with_obstacle(
        const float center_height = 0.0F) {
    std::vector<Eigen::Vector3f> scan;
    for (int x = 0; x < 3; ++x) {
        for (int y = 0; y < 3; ++y) {
            const float height = x == 1 && y == 1 ? center_height : 0.0F;
            scan.emplace_back(
                0.1F + 0.2F * static_cast<float>(x),
                0.1F + 0.2F * static_cast<float>(y),
                height);
        }
    }
    scan.emplace_back(0.3F, 0.3F, 0.50F);
    return scan;
}

TEST(PointcloudMapping, ContinuousStaticReturnsBecomePersistent) {
    auto mapping = make_mapping();
    const std::vector<Eigen::Vector3f> scan{{1.0F, 2.0F, 0.17F}};
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    EXPECT_TRUE(mapping.get_snapshot().points.empty());
    mapping.add_pointcloud(scan);
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 1U);
}

TEST(PointcloudMapping, ObservationGapRestartsConfirmation) {
    auto mapping = make_mapping();
    const std::vector<Eigen::Vector3f> scan{{1.0F, 2.0F, 0.17F}};
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud({});
    mapping.add_pointcloud({});
    mapping.add_pointcloud({});
    mapping.add_pointcloud(scan);
    EXPECT_TRUE(mapping.get_snapshot().points.empty());
}

TEST(PointcloudMapping, MovingReturnsDoNotBecomePersistent) {
    auto mapping = make_mapping();
    mapping.add_pointcloud({Eigen::Vector3f{0.0F, 0.0F, 0.4F}});
    mapping.add_pointcloud({Eigen::Vector3f{0.3F, 0.0F, 0.4F}});
    mapping.add_pointcloud({Eigen::Vector3f{0.6F, 0.0F, 0.4F}});
    EXPECT_TRUE(mapping.get_snapshot().points.empty());
}

TEST(PointcloudMapping, ConfirmedStaticReturnsSurviveRevisitAndPruning) {
    auto mapping = make_mapping();
    const std::vector<Eigen::Vector3f> scan{{1.01F, 2.01F, 0.17F}};
    for (int frame = 0; frame < 3; ++frame) mapping.add_pointcloud(scan);
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 1U);
    for (int frame = 0; frame < 30; ++frame) mapping.add_pointcloud({});
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 1U);
    mapping.add_pointcloud({Eigen::Vector3f{1.09F, 2.01F, 0.17F}});
    const auto snapshot = mapping.get_snapshot();
    ASSERT_EQ(snapshot.points.size(), 1U);
    EXPECT_EQ(snapshot.persistent_voxels, 1U);
    // A single revisit must not replace the confirmed surface centroid.
    EXPECT_NEAR(snapshot.points.front().x(), 1.03F, 1.0e-5F);
    for (int frame = 0; frame < 30; ++frame) mapping.add_pointcloud({});
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 1U);
}

TEST(PointcloudMapping, StrictConfirmedReturnsSurviveRevisit) {
    auto mapping = make_filter_mode_mapping(
        util::PointcloudMappingFilterMode::Strict);
    const std::vector<Eigen::Vector3f> scan{{1.0F, 2.0F, 0.4F}};
    for (int frame = 0; frame < 6; ++frame) mapping.add_pointcloud(scan);
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 1U);
    for (int frame = 0; frame < 30; ++frame) mapping.add_pointcloud({});
    mapping.add_pointcloud(scan);
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 1U);
}

TEST(PointcloudMapping, ConfirmedGroundSurvivesSparseRevisit) {
    auto mapping = make_ground_mapping();
    const auto scan = flat_ground_with_obstacle();
    for (int frame = 0; frame < 3; ++frame) mapping.add_pointcloud(scan);
    ASSERT_EQ(mapping.get_snapshot().persistent_voxels, 9U);
    for (int frame = 0; frame < 30; ++frame) mapping.add_pointcloud({});
    // This lone return cannot pass the current-frame ground-neighbour test.
    mapping.add_pointcloud({Eigen::Vector3f{0.3F, 0.3F, 0.0F}});
    const auto snapshot = mapping.get_snapshot();
    EXPECT_EQ(snapshot.persistent_voxels, 9U);
    EXPECT_EQ(snapshot.ground_protected_voxels, 9U);
    EXPECT_EQ(mapping.get_snapshot(4).persistent_voxels, 9U);
    for (int frame = 0; frame < 30; ++frame) mapping.add_pointcloud({});
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 9U);
}

TEST(PointcloudMapping, ConfirmedGroundSurvivesClassificationChangeWithoutGap) {
    auto mapping = make_ground_mapping();
    const auto scan = flat_ground_with_obstacle();
    for (int frame = 0; frame < 3; ++frame) mapping.add_pointcloud(scan);
    mapping.add_pointcloud({Eigen::Vector3f{0.3F, 0.3F, 0.0F}});
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 9U);
    EXPECT_EQ(mapping.get_snapshot().ground_protected_voxels, 9U);
}

TEST(PointcloudMapping, SparseCandidateRevisitsNeverAccumulateConfirmation) {
    auto mapping = make_mapping();
    const std::vector<Eigen::Vector3f> candidate{{1.0F, 2.0F, 0.4F}};
    for (int visit = 0; visit < 10; ++visit) {
        mapping.add_pointcloud(candidate);
        for (int frame = 0; frame < 3; ++frame) mapping.add_pointcloud({});
        EXPECT_TRUE(mapping.get_snapshot().points.empty());
    }
    for (int frame = 0; frame < 30; ++frame) mapping.add_pointcloud({});
    EXPECT_EQ(mapping.get_snapshot().tracked_voxels, 0U);
}

TEST(PointcloudMapping, StableGroundAndSeventeenCentimeterStepArePreserved) {
    auto mapping = make_mapping();
    const std::vector<Eigen::Vector3f> scan{
        {0.0F, 0.0F, 0.0F},
        {0.0F, 0.0F, 0.17F}};
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    EXPECT_EQ(mapping.get_snapshot().persistent_voxels, 2U);
}

TEST(PointcloudMapping, GroundUsesFastPathButElevatedObstacleDoesNot) {
    auto mapping = make_ground_mapping();
    const auto scan = flat_ground_with_obstacle();
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    EXPECT_TRUE(mapping.get_snapshot().points.empty());
    mapping.add_pointcloud(scan);
    const auto snapshot = mapping.get_snapshot();
    EXPECT_EQ(snapshot.persistent_voxels, 9U);
    EXPECT_EQ(snapshot.ground_protected_voxels, 9U);
}

TEST(PointcloudMapping, StepEdgeDoesNotUseGroundFastPath) {
    auto mapping = make_ground_mapping();
    const auto scan = flat_ground_with_obstacle(0.17F);
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    const auto snapshot = mapping.get_snapshot();
    EXPECT_GT(snapshot.ground_protected_voxels, 0U);
    for (const auto &point : snapshot.points) {
        EXPECT_LT(point.z(), 0.10F);
    }
}

TEST(PointcloudMapping, BypassPersistsAFirstObservationWithoutGroundFastPath) {
    auto mapping = make_filter_mode_mapping(
        util::PointcloudMappingFilterMode::Bypass);
    mapping.add_pointcloud({Eigen::Vector3f{1.0F, 2.0F, 0.40F}});
    const auto snapshot = mapping.get_snapshot();
    EXPECT_EQ(snapshot.filter_mode, util::PointcloudMappingFilterMode::Bypass);
    EXPECT_EQ(snapshot.persistent_voxels, 1U);
    EXPECT_EQ(snapshot.ground_protected_voxels, 0U);
}

TEST(PointcloudMapping, StrictDoesNotUseTheGroundFastPath) {
    auto mapping = make_filter_mode_mapping(
        util::PointcloudMappingFilterMode::Strict);
    const auto scan = flat_ground_with_obstacle();
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    EXPECT_TRUE(mapping.get_snapshot().points.empty());
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    mapping.add_pointcloud(scan);
    const auto snapshot = mapping.get_snapshot();
    EXPECT_EQ(snapshot.filter_mode, util::PointcloudMappingFilterMode::Strict);
    EXPECT_EQ(snapshot.persistent_voxels, 10U);
    EXPECT_EQ(snapshot.ground_protected_voxels, 0U);
}

TEST(PointcloudMapping, FilterModeParserFallsBackSafely) {
    bool valid = false;
    EXPECT_EQ(
        util::pointcloud_mapping_filter_mode_from_string("bypass", &valid),
        util::PointcloudMappingFilterMode::Bypass);
    EXPECT_TRUE(valid);
    EXPECT_EQ(
        util::pointcloud_mapping_filter_mode_from_string("strict", &valid),
        util::PointcloudMappingFilterMode::Strict);
    EXPECT_TRUE(valid);
    EXPECT_EQ(
        util::pointcloud_mapping_filter_mode_from_string("typo", &valid),
        util::PointcloudMappingFilterMode::Protected);
    EXPECT_FALSE(valid);
    EXPECT_STREQ(
        util::pointcloud_mapping_filter_mode_name(
            util::PointcloudMappingFilterMode::Protected),
        "protected");
}

}  // namespace
