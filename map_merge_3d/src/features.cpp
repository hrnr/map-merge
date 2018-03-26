#include <map_merge_3d/features.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
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

LocalDescriptorsPtr computeLocalDescriptors(const PointCloudPtr &points,
                                            const SurfaceNormalsPtr &normals,
                                            const PointCloudPtr &keypoints,
                                            double feature_radius)
{
  pcl::PFHEstimation<PointT, NormalT, LocalDescriptorT> descriptor;
  descriptor.setRadiusSearch(feature_radius);
  descriptor.setSearchSurface(points);
  descriptor.setInputNormals(normals);
  descriptor.setInputCloud(keypoints);

  LocalDescriptorsPtr local_descriptors(new LocalDescriptors);
  descriptor.compute(*local_descriptors);

  return local_descriptors;
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
