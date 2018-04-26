#include <map_merge_3d/features.h>
#include "dispatch_descriptors.h"

#include <algorithm>

#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_representation.h>

namespace map_merge_3d
{
PointCloudPtr downSample(const PointCloudConstPtr &input, double resolution)
{
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(float(resolution), float(resolution), float(resolution));
  filter.setInputCloud(input);

  PointCloudPtr output(new PointCloud);
  filter.filter(*output);

  return output;
}

/* Use a RadiusOutlierRemoval filter to remove all points with too few local
 * neighbors */
PointCloudPtr removeOutliers(const PointCloudConstPtr &input, double radius,
                             int min_neighbors)
{
  pcl::RadiusOutlierRemoval<PointT> filter;
  filter.setInputCloud(input);
  filter.setRadiusSearch(radius);
  filter.setMinNeighborsInRadius(min_neighbors);

  PointCloudPtr output(new PointCloud);
  filter.filter(*output);

  return output;
}

static PointCloudPtr detectKeypointsSIFT(const PointCloudConstPtr &points,
                                         double min_scale, int nr_octaves,
                                         int nr_scales_per_octave,
                                         double min_contrast)
{
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> detector;
  detector.setScales(float(min_scale), nr_octaves, nr_scales_per_octave);
  detector.setMinimumContrast(float(min_contrast));
  detector.setInputCloud(points);

  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  detector.compute(keypoints_temp);

  PointCloudPtr keypoints(new PointCloud);
  pcl::copyPointCloud(keypoints_temp, *keypoints);

  return keypoints;
}

static PointCloudPtr detectKeypointsHarris(const PointCloudConstPtr &points,
                                           const SurfaceNormalsPtr &normals,
                                           double threshold, double radius)
{
  pcl::HarrisKeypoint3D<PointT, pcl::PointXYZI> detector;
  detector.setInputCloud(points);
  detector.setNormals(normals);
  detector.setNonMaxSupression(true);
  detector.setRefine(true);
  detector.setThreshold(float(threshold));
  detector.setRadius(float(radius));

  pcl::PointCloud<pcl::PointXYZI> keypoints_temp;
  detector.compute(keypoints_temp);

  PointCloudPtr keypoints(new PointCloud);
  pcl::copyPointCloud(keypoints_temp, *keypoints);

  return keypoints;
}

PointCloudPtr detectKeypoints(const PointCloudConstPtr &points,
                              const SurfaceNormalsPtr &normals, Keypoint type,
                              double threshold, double radius,
                              double resolution)
{
  switch (type) {
    case Keypoint::SIFT:
      return detectKeypointsSIFT(points, resolution, 3, 3, threshold);
    case Keypoint::HARRIS:
      return detectKeypointsHarris(points, normals, threshold, radius);
  }
}

/* implementation for specific descriptor type  */
template <typename DescriptorExtractor, typename DescriptorT>
static LocalDescriptorsPtr
computeLocalDescriptors(const PointCloudConstPtr &points,
                        const SurfaceNormalsPtr &normals,
                        const PointCloudPtr &keypoints, double feature_radius)
{
  DescriptorExtractor descriptor;
  descriptor.setRadiusSearch(feature_radius);
  descriptor.setSearchSurface(points);
  descriptor.setInputNormals(normals);
  descriptor.setInputCloud(keypoints);

  typename pcl::PointCloud<DescriptorT>::Ptr descriptors(
      new pcl::PointCloud<DescriptorT>);
  descriptor.compute(*descriptors);

  // remove invalid descriptors (it might not be possible to compute descriptors
  // for all keypoints)

  // find invalid points
  pcl::DefaultPointRepresentation<DescriptorT> point_rep;
  pcl::IndicesPtr invalid_indices(new std::vector<int>);
  int i = 0;
  for (auto d : *descriptors) {
    if (!point_rep.isValid(d)) {
      invalid_indices->push_back(i);
    }
    ++i;
  }

  // remove invalid descriptors
  pcl::ExtractIndices<DescriptorT> filter;
  filter.setInputCloud(descriptors);
  filter.setIndices(invalid_indices);
  filter.setNegative(true);
  filter.filter(*descriptors);

  // filter also keypoints to keep keypoints and the descriptors synchronized
  pcl::ExtractIndices<PointT> filter_keypoints;
  filter_keypoints.setInputCloud(keypoints);
  filter_keypoints.setIndices(invalid_indices);
  filter_keypoints.setNegative(true);
  filter_keypoints.filter(*keypoints);

  assert(keypoints->size() == descriptors->size());

  // convert to PointCloud2 which is able to hold any descriptors data
  LocalDescriptorsPtr result(new LocalDescriptors);
  pcl::toPCLPointCloud2(*descriptors, *result);

  return result;
}

LocalDescriptorsPtr computeLocalDescriptors(const PointCloudConstPtr &points,
                                            const SurfaceNormalsPtr &normals,
                                            const PointCloudPtr &keypoints,
                                            Descriptor descriptor,
                                            double feature_radius)
{
  // this will be dispatched for all descriptors type
  auto functor = [&](auto descriptor_type) {
    return computeLocalDescriptors<
        typename decltype(descriptor_type)::Estimator,
        typename decltype(descriptor_type)::PointType>(
        points, normals, keypoints, feature_radius);
  };
  return dispatchForEachDescriptor(descriptor, functor);
}

SurfaceNormalsPtr computeSurfaceNormals(const PointCloudConstPtr &input,
                                        double radius)
{
  pcl::NormalEstimation<PointT, NormalT> estimator;
  estimator.setRadiusSearch(radius);
  estimator.setInputCloud(input);

  SurfaceNormalsPtr normals(new SurfaceNormals);
  estimator.compute(*normals);

  return normals;
}

}  // namespace map_merge_3d
