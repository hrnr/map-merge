#ifndef MAP_MERGE_MATCHING_H_
#define MAP_MERGE_MATCHING_H_

#include <map_merge_3d/typedefs.h>

CorrespondencesPtr
findFeatureCorrespondences(const LocalDescriptorsPtr &source_descriptors,
                           const LocalDescriptorsPtr &target_descriptors,
                           size_t k = 5);

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

/**
 * @brief Use ICP to estimate transform between grids
 *
 * @param source_points source point cloud
 * @param target_points target point cloud. Source point cloud will be aligned
 * to this one.
 * @param initial_guess Initial transfromation to start from
 *
 * @param max_correspondence_distance A threshold on the distance between any
 * two corresponding points.  Any corresponding points that are further apart
 * than this threshold will be ignored when computing the source-to-target
 * transformation
 * @param outlier_rejection_threshold A threshold used to define outliers during
 * RANSAC
 * @param max_iterations maximum iterations for RANSAC
 * @param transformation_epsilon The smallest iterative transformation allowed
 * before the algorithm is considered to have converged
 *
 * @return estiamted transformation between two grids.
 */
Eigen::Matrix4f estimateTransformICP(const PointCloudPtr &source_points,
                                     const PointCloudPtr &target_points,
                                     const Eigen::Matrix4f &initial_guess,
                                     double max_correspondence_distance,
                                     double outlier_rejection_threshold,
                                     int max_iterations = 100,
                                     double transformation_epsilon = 0.0);

#endif  // MAP_MERGE_MATCHING_H_
