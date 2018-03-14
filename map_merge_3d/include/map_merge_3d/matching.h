#ifndef MAP_MERGE_MATCHING_H_
#define MAP_MERGE_MATCHING_H_

#include <map_merge_3d/typedefs.h>

CorrespondencesPtr
findFeatureCorrespondences(const LocalDescriptorsPtr &source_descriptors,
                           const LocalDescriptorsPtr &target_descriptors);

Eigen::Matrix4f estimateTransformFromCorrespondences(
    const PointCloudPtr &source_keypoints,
    const PointCloudPtr &target_keypoints,
    const CorrespondencesPtr &correspondences, CorrespondencesPtr &inliers,
    double inlier_threshold);

#endif  // MAP_MERGE_MATCHING_H_
