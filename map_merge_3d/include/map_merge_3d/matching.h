#ifndef MAP_MERGE_MATCHING_H_
#define MAP_MERGE_MATCHING_H_

#include <map_merge_3d/enum.h>
#include <map_merge_3d/typedefs.h>

namespace map_merge_3d
{
/**
 * @defgroup matching Matching
 * @brief Low-level functions for matching the map pair
 * @details Low-level functions to find correspondences between two pointclouds
 * and match them together. These functions operate on two pointclouds.
 * @{
 */

/**
 * @brief Finds correspondences between two sets of feature descriptors
 * @details Uses cross-matching algorithm with first-k selection
 *
 * @param source_descriptors Feature descriptors of source pointcloud
 * @param target_descriptors Feature descriptors of target pointcloud
 * @param k number of nearest descriptors to consider for matching
 * @return correspondences source -> target
 */
CorrespondencesPtr findFeatureCorrespondences(
    const LocalDescriptorsPtr &source_descriptors,
    const LocalDescriptorsPtr &target_descriptors, size_t k = 5);

/**
 * @brief Estimates transformation between source and target pointcloud based on
 * correspondences
 * @details Uses RANSAC to find inliers fitting a rigid transformation model.
 * Final transformation is found using SVD on inliers set.
 *
 * @param source_keypoints Keypoints of source pointcloud
 * @param target_keypoints Keypoints of target pointcloud
 * @param correspondences Correspondences between keypoints
 * @param[out] inliers Estimated inliers from RANSAC
 * @param inlier_threshold threshold for considering a point as inlier in RANSAC
 * @return estimated rigid transformation between  source and target or zero
 * matrix if transformation could not be estimated
 */
Eigen::Matrix4f estimateTransformFromCorrespondences(
    const PointCloudPtr &source_keypoints,
    const PointCloudPtr &target_keypoints,
    const CorrespondencesPtr &correspondences, CorrespondencesPtr &inliers,
    double inlier_threshold);

/**
 * @brief Estimates transformation between source and target pointcloud based on
 * descriptors
 * @details Use SampleConsensusInitialAlignment to find a rough alignment from
 * the source to the target
 *
 * @param source_keypoints Keypoints of source pointcloud
 * @param source_descriptors descriptors for source_keypoints
 * @param target_keypoints Keypoints of target pointcloud
 * @param target_descriptors descriptors for target_keypoints
 * @param min_sample_distance The minimum distance between any two random
 * samples
 * @param max_correspondence_distance Maximum distance for a matched points to
 * be considered the same point
 * @param max_iterations The number of RANSAC iterations to perform
 * @return estimated rigid transformation between source and target or zero
 * matrix
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
 * @param target_points target point cloud.
 * @param initial_guess Initial transformation to start with
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
 * @return estimated rigid transformation between source and target pointclouds
 */
Eigen::Matrix4f estimateTransformICP(const PointCloudPtr &source_points,
                                     const PointCloudPtr &target_points,
                                     const Eigen::Matrix4f &initial_guess,
                                     double max_correspondence_distance,
                                     double outlier_rejection_threshold,
                                     int max_iterations = 100,
                                     double transformation_epsilon = 0.0);

// defines enum class EstimationMethod + string conversions
ENUM_CLASS(EstimationMethod, MATCHING, SAC_IA);

/**
 * @brief Estimate transformation between two pointclouds
 * @details Uses extracted features to estimate rigid transformation. First the
 * initial transformation is estimated using selected method, then the
 * transformation may be selectively refined using ICP.
 *
 * @param source_points Source pointcloud
 * @param source_keypoints Keypoints of source pointcloud
 * @param source_descriptors Descriptors for keypoints of source pointcloud
 * @param target_points Target pointcloud
 * @param target_keypoints Keypoints of target pointcloud
 * @param target_descriptors Descriptors for keypoints of target pointcloud
 * @param method Method for estimating initial transformation
 * @param refine Whether to refine initial transformation with ICP.
 * @param inlier_threshold Threshold for inliers in RANSAC during initial
 * estimation.
 * @param max_correspondence_distance Maximum distance for a matched points to
 * be considered the same point
 * @param max_iterations maximum iterations for RANSAC
 * @param matching_k number of nearest descriptors to consider for matching
 * @param transform_epsilon the smallest change allowed until ICP convergence.
 * @return estimated rigid transform between source and target pointclouds or
 * zero matrix if the transformation could not be estimated
 */
Eigen::Matrix4f estimateTransform(
    const PointCloudPtr &source_points, const PointCloudPtr &source_keypoints,
    const LocalDescriptorsPtr &source_descriptors,
    const PointCloudPtr &target_points, const PointCloudPtr &target_keypoints,
    const LocalDescriptorsPtr &target_descriptors, EstimationMethod method,
    bool refine, double inlier_threshold, double max_correspondence_distance,
    int max_iterations, size_t matching_k, double transform_epsilon);

/**
 * @brief Computes euclidean distance between two pointclouds.
 * @details Computes a euclidean score for an estimated transformation. Because
 * we expect only some parts of the maps overlapping, only points closer than
 * max_distance will be include in the score.
 *
 * @param source_points Source pointcloud
 * @param target_points Target pointcloud
 * @param transform estimated transformation between source and target
 * @param max_distance Maximum distance between two points to be included in the
 * score.
 * @return transformation euclidean score
 */
double transformScore(const PointCloudPtr &source_points,
                      const PointCloudPtr &target_points,
                      const Eigen::Matrix4f &transform, double max_distance);

///@} group matching

}  // namespace map_merge_3d

#endif  // MAP_MERGE_MATCHING_H_
