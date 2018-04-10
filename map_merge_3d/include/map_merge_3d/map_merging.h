#ifndef MAP_MERGE_MAP_MERGING_H_
#define MAP_MERGE_MAP_MERGING_H_

#include <map_merge_3d/features.h>
#include <map_merge_3d/matching.h>
#include <map_merge_3d/typedefs.h>

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
};

std::vector<Eigen::Matrix4f> estimateMapsTransforms(
    const std::vector<PointCloudPtr> &clouds, const MapMergingParams &params);

PointCloudPtr composeMaps(const std::vector<PointCloudPtr> &clouds,
                          const std::vector<Eigen::Matrix4f> &transforms,
                          double resolution);

#endif  // MAP_MERGE_MAP_MERGING_H_
