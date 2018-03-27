#include <map_merge_3d/features.h>

#include <pcl/conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/keypoints/sift_keypoint.h>

PointCloudPtr downSample(const PointCloudPtr &input, double resolution)
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
PointCloudPtr removeOutliers(const PointCloudPtr &input, double radius,
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

PointCloudPtr detectKeypoints(const PointCloudPtr &points, double min_scale,
                              int nr_octaves, int nr_scales_per_octave,
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

/* implementation for specific descriptor type  */
template <template<typename,typename,typename> class DescriptorExtractor, typename DescriptorT>
LocalDescriptorsPtr computeLocalDescriptors(const PointCloudPtr &points,
                                            const SurfaceNormalsPtr &normals,
                                            const PointCloudPtr &keypoints,
                                            double feature_radius)
{
  DescriptorExtractor<PointT, NormalT, DescriptorT> descriptor;
  descriptor.setRadiusSearch(feature_radius);
  descriptor.setSearchSurface(points);
  descriptor.setInputNormals(normals);
  descriptor.setInputCloud(keypoints);

  pcl::PointCloud<DescriptorT> descriptors;
  descriptor.compute(descriptors);

  // convert to PointCloud2 which is able to hold any descriptors data
  LocalDescriptorsPtr result(new LocalDescriptors);
  pcl::toPCLPointCloud2(descriptors, *result);

  return result;
}

LocalDescriptorsPtr computeLocalDescriptors(const PointCloudPtr &points,
                                            const SurfaceNormalsPtr &normals,
                                            const PointCloudPtr &keypoints,
                                            Descriptor descriptor,
                                            double feature_radius)
{
  switch (descriptor) {
    case Descriptor::PFH:
      return computeLocalDescriptors<pcl::PFHEstimation, pcl::PFHSignature125>(
          points, normals, keypoints, feature_radius);
  }
}

SurfaceNormalsPtr computeSurfaceNormals(const PointCloudPtr &input,
                                        double radius)
{
  pcl::NormalEstimation<PointT, NormalT> estimator;
  estimator.setRadiusSearch(radius);
  estimator.setInputCloud(input);

  SurfaceNormalsPtr normals(new SurfaceNormals);
  estimator.compute(*normals);

  return normals;
}
