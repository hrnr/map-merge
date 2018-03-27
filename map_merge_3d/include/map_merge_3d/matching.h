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

/**
 * @brief Use SampleConsensusInitialAlignment to find a rough alignment from the
 * source to the target
 *
 * @param source_keypoints keypoints1
 * @param source_descriptors descriptors for keypoints1
 * @param target_keypoints keypoints2
 * @param target_descriptors descriptors for keypoints2
 * @param min_sample_distance The minimum distance between any two random
 * samples
 * @param max_correspondence_distance [description]
 * @param max_iterations The number of RANSAC iterations to perform
 * @return estimated transform between 2 keypoints sets
 */
Eigen::Matrix4f estimateTransformFromDescriptorsSets(
    const PointCloudPtr &source_keypoints,
    const LocalDescriptorsPtr &source_descriptors,
    const PointCloudPtr &target_keypoints,
    const LocalDescriptorsPtr &target_descriptors, double min_sample_distance,
    double max_correspondence_distance, int max_iterations);

#endif  // MAP_MERGE_MATCHING_H_
