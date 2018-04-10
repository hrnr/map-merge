#ifndef MAP_MERGE_MAP_MERGING_H_
#define MAP_MERGE_MAP_MERGING_H_

#include <map_merge_3d/features.h>
#include <map_merge_3d/matching.h>
#include <map_merge_3d/typedefs.h>

struct MapMergingParams {
  double resolution;
  double descriptor_radius;
  int outliers_min_neighbours;
  double normal_radius;
  Keypoint keypoint_type;
  double keypoint_threshold;
  Descriptor descriptor_type;
  EstimationMethod estimation_method;
  bool refine_transform;
  double inlier_threshold;
  double max_correspondence_distance;
  int max_iterations;
  size_t matching_k;
  double transform_epsilon;
  double confidence_threshold;
  double output_resolution;
};

std::vector<Eigen::Matrix4f> estimateMapsTransforms(
    const std::vector<PointCloudPtr> &clouds, const MapMergingParams &params);

PointCloudPtr composeMaps(const std::vector<PointCloudPtr> &clouds,
                          const std::vector<Eigen::Matrix4f> &transforms,
                          double resolution);

#endif  // MAP_MERGE_MAP_MERGING_H_
