#include <map_merge_3d/features.h>
#include <map_merge_3d/map_merging.h>

std::vector<Eigen::Matrix4f> estimateMapsTransforms(
    const std::vector<PointCloudPtr> &clouds, const MapMergingParams &params)
{
  // per cloud data extracted for transform estimation
  std::vector<PointCloudPtr> clouds_resized;
  std::vector<SurfaceNormalsPtr> normals;
  std::vector<PointCloudPtr> keypoints;
  std::vector<LocalDescriptorsPtr> descriptors;
  clouds_resized.reserve(clouds.size());
  normals.reserve(clouds.size());
  keypoints.reserve(clouds.size());
  descriptors.reserve(clouds.size());

  /* compute per-cloud features */

  // resize clouds to registration resolution
  for (auto &cloud : clouds) {
    PointCloudPtr resized = downSample(cloud, params.resolution);
    clouds_resized.emplace_back(std::move(resized));
  }

  // remove noise (this reduces number of keypoints)
  for (auto &cloud : clouds_resized) {
    cloud = removeOutliers(cloud, params.descriptor_radius,
                           params.outliers_min_neighbours);
  }

  // compute normals
  for (const auto &cloud : clouds_resized) {
    auto cloud_normals = computeSurfaceNormals(cloud, params.normal_radius);
    normals.emplace_back(std::move(cloud_normals));
  }

  // detect keypoints
  for (size_t i = 0; i < clouds_resized.size(); ++i) {
    auto cloud_keypoints = detectKeypoints(
        clouds_resized[i], normals[i], params.keypoint_type,
        params.keypoint_threshold, params.normal_radius, params.resolution);
    keypoints.emplace_back(std::move(cloud_keypoints));
  }

  for (size_t i = 0; i < clouds_resized.size(); ++i) {
    auto cloud_descriptors = computeLocalDescriptors(
        clouds_resized[i], normals[i], keypoints[i], params.descriptor_type,
        params.descriptor_radius);
    descriptors.emplace_back(std::move(cloud_descriptors));
  }

  /* estimate pairwise transforms */

  // TODO

  return {};
}
