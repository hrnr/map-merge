#ifndef MAP_MERGE_MAP_MERGING_H_
#define MAP_MERGE_MAP_MERGING_H_

#include <ostream>

#include <map_merge_3d/features.h>
#include <map_merge_3d/matching.h>
#include <map_merge_3d/typedefs.h>

#include <ros/ros.h>

namespace map_merge_3d
{
/**
 * @defgroup map_merging Map merging
 * @brief High-level map-merging interface.
 * @details High-level interface to estimate transformations between n
 * pointclouds and compositing global map..
 * @{
 */

/**
 * @brief Parameters for map merging high-level interface
 * @details Contains all tunables for estimating transformation between n maps
 * and for compositing the global map
 *
 */
struct MapMergingParams {
  double resolution = 0.1;
  double descriptor_radius = resolution * 8.0;
  int outliers_min_neighbours = 50;
  double normal_radius = resolution * 6.0;
  Keypoint keypoint_type = Keypoint::SIFT;
  double keypoint_threshold = 5.0;
  Descriptor descriptor_type = Descriptor::PFH;
  EstimationMethod estimation_method = EstimationMethod::MATCHING;
  bool refine_transform = true;
  double inlier_threshold = resolution * 5.0;
  double max_correspondence_distance = inlier_threshold * 2.0;
  int max_iterations = 500;
  size_t matching_k = 5;
  double transform_epsilon = 1e-2;
  double confidence_threshold = 0.0;
  double output_resolution = 0.05;

  /**
   * @brief Sources parameters from command line arguments
   * @details Uses PCL's command line parser to initialize the parameters.
   * Format is `--param_name <value>`. param_name is the same as the struct
   * member.
   *
   * @param argc arguments count
   * @param argv program arguments
   *
   * @return parameters with values from command line of default values where
   * not provided.
   */
  static MapMergingParams fromCommandLine(int argc, char **argv);

  /**
   * @brief Sources parameters from ROS node parameters
   * @details Parameter names are the same as the struct
   * member.
   *
   * @param node ROS node to source parameters from
   * @return parameters with values from ROS params of default values where
   * not provided.
   */
  static MapMergingParams fromROSNode(const ros::NodeHandle &node);
};
std::ostream &operator<<(std::ostream &stream, const MapMergingParams &params);

/**
 * @brief Estimate transformations between n pointclouds
 * @details Estimation is based on overlapping space. One of the pointclouds
 * will be selected as the reference frame for all the transformations.
 *
 * @param clouds input pointclouds
 * @param params parameters for estimation
 *
 * @return Estimated transformations pointcloud -> reference frame for each
 * input pointcloud. If the transformation could not estimated, the
 * transformation will be zero matrix for the respective pointcloud.
 */
std::vector<Eigen::Matrix4f>
estimateMapsTransforms(const std::vector<PointCloudConstPtr> &clouds,
                       const MapMergingParams &params);

/**
 * @brief Composes the global map
 * @details Pointclouds with zero transformation will be skipped.
 *
 * @param clouds input clouds
 * @param transforms estimated transformations between input clouds
 * @param resolution resolution of the output pointcloud
 *
 * @return the global map or nullptr if the input is empty
 */
PointCloudPtr composeMaps(const std::vector<PointCloudConstPtr> &clouds,
                          const std::vector<Eigen::Matrix4f> &transforms,
                          double resolution);

///@} group map_merging

}  // namespace map_merge_3d

#endif  // MAP_MERGE_MAP_MERGING_H_
