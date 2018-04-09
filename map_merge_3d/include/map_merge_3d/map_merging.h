#ifndef MAP_MERGE_MAP_MERGING_H_
#define MAP_MERGE_MAP_MERGING_H_

#include <map_merge_3d/typedefs.h>
#include <map_merge_3d/features.h>

struct MapMergingParams {
  double resolution;
  double descriptor_radius;
  int outliers_min_neighbours;
  double normal_radius;
  Keypoint keypoint_type;
  double keypoint_threshold;
  Descriptor descriptor_type;
};

std::vector<Eigen::Matrix4f> estimateMapsTransforms(
    const std::vector<PointCloudPtr> &clouds, const MapMergingParams &params);

#endif  // MAP_MERGE_MAP_MERGING_H_
