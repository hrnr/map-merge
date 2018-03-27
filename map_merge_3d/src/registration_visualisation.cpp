#include <map_merge_3d/typedefs.h>

#include <map_merge_3d/features.h>
#include <map_merge_3d/matching.h>
#include <map_merge_3d/visualise.h>

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

/* Use IterativeClosestPoint to find a precise alignment from the source cloud
 * to the target cloud,
 * starting with an intial guess
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align
 * with the target point cloud
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud
 * will be aligned
 *   intial_alignment
 *     An initial estimate of the transformation matrix that aligns the source
 * points to the target points
 *   max_correspondence_distance
 *     A threshold on the distance between any two corresponding points.  Any
 * corresponding points that are further
 *     apart than this threshold will be ignored when computing the
 * source-to-target transformation
 *   outlier_rejection_threshold
 *     A threshold used to define outliers during RANSAC outlier rejection
 *   transformation_epsilon
 *     The smallest iterative transformation allowed before the algorithm is
 * considered to have converged
 *   max_iterations
 *     The maximum number of ICP iterations to perform
 * Return: A transformation matrix that will precisely align the points in
 * source to the points in target
 */
Eigen::Matrix4f refineAlignment(const PointCloudPtr &source_points,
                                const PointCloudPtr &target_points,
                                const Eigen::Matrix4f &initial_alignment,
                                float max_correspondence_distance,
                                float outlier_rejection_threshold,
                                float transformation_epsilon,
                                float max_iterations)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp.setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
  icp.setTransformationEpsilon(transformation_epsilon);
  icp.setMaximumIterations(max_iterations);

  PointCloudPtr source_points_transformed(new PointCloud);
  pcl::transformPointCloud(*source_points, *source_points_transformed,
                           initial_alignment);

  icp.setInputCloud(source_points_transformed);
  icp.setInputTarget(target_points);

  PointCloud registration_output;
  icp.align(registration_output);

  return (icp.getFinalTransformation() * initial_alignment);
}

void estimateTransformICP(PointCloudPtr cloud1, PointCloudPtr cloud2,
                          PointCloudPtr output, int max_iterations,
                          double max_correspondence_distance)
{
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputTarget(cloud1);
  icp.setInputSource(cloud2);
  icp.setMaxCorrespondenceDistance(max_correspondence_distance * 2);
  icp.setRANSACOutlierRejectionThreshold(max_correspondence_distance);
  icp.setMaximumIterations(max_iterations);
  icp.setTransformationEpsilon(1e-8);

  if (!output) {
    output = PointCloudPtr(new PointCloud);
  }
  icp.align(*output);

  std::cout << "Final transformation: " << std::endl
            << icp.getFinalTransformation() << std::endl;
  if (icp.hasConverged()) {
    std::cout << "ICP converged." << std::endl
              << "The score is " << icp.getFitnessScore() << std::endl;
  } else {
    std::cout << "ICP did not converge.";
  }
}

void printPointCloud2Summary(const pcl::PCLPointCloud2 &v)
{
  std::cout << "header: " << std::endl;
  std::cout << v.header;
  std::cout << "height: ";
  std::cout << "  " << v.height << std::endl;
  std::cout << "width: ";
  std::cout << "  " << v.width << std::endl;
  std::cout << "fields[]" << std::endl;
  for (size_t i = 0; i < v.fields.size(); ++i) {
    std::cout << "  fields[" << i << "]: ";
    std::cout << std::endl;
    std::cout << "    " << v.fields[i] << std::endl;
  }
  std::cout << "is_bigendian: ";
  std::cout << "  " << v.is_bigendian << std::endl;
  std::cout << "point_step: ";
  std::cout << "  " << v.point_step << std::endl;
  std::cout << "row_step: ";
  std::cout << "  " << v.row_step << std::endl;
  std::cout << "is_dense: ";
  std::cout << "  " << v.is_dense << std::endl;
}

int main(int argc, char **argv)
{
  double resolution = 0.1;
  int min_neighbours = 50;
  const int nr_octaves = 3;
  const int nr_octaves_per_scale = 3;
  const double min_contrast = 5.0;
  double normal_radius = resolution * 6.;
  double descriptor_radius = resolution * 8.;
  double filtering_radius = resolution * 2.;
  const double inlier_threshold = resolution * 5.;
  const double max_correspondence_distance = inlier_threshold * 2.;
  const int nr_iterations = 1000;
  const int max_iterations = 200;

  pcl::console::parse_argument(argc, argv, "--normal_radius", normal_radius);
  pcl::console::parse_argument(argc, argv, "--descriptor_radius",
                               descriptor_radius);
  pcl::console::parse_argument(argc, argv, "--resolution", resolution);
  pcl::console::parse_argument(argc, argv, "--min_neighbours", min_neighbours);
  pcl::console::parse_argument(argc, argv, "--filtering_radius",
                               filtering_radius);

  PointCloudPtr cloud1(new PointCloud);
  PointCloudPtr cloud2(new PointCloud);

  // Load object and scene
  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud1) < 0 ||
      pcl::io::loadPCDFile<PointT>(argv[2], *cloud2) < 0) {
    pcl::console::print_error("Error loading object/scene file!\n");
    return (1);
  }
  pcl::console::print_highlight("Pointclouds loaded.\n");

  cloud1 = downSample(cloud1, resolution);
  cloud2 = downSample(cloud2, resolution);

  visualisePointCloud(cloud1);

  cloud1 = removeOutliers(cloud1, descriptor_radius, min_neighbours);
  cloud2 = removeOutliers(cloud2, descriptor_radius, min_neighbours);

  visualisePointCloud(cloud1);

  /* detect keypoints */
  PointCloudPtr keypoints1, keypoints2;
  {
    pcl::ScopeTime t("feature detection");
    keypoints1 = detectKeypoints(cloud1, resolution, nr_octaves,
                                 nr_octaves_per_scale, min_contrast);
    keypoints2 = detectKeypoints(cloud2, resolution, nr_octaves,
                                 nr_octaves_per_scale, min_contrast);
  }

  visualiseKeypoints(cloud1, keypoints1);

  /* compute descriptors */
  LocalDescriptorsPtr descriptors1, descriptors2;
  SurfaceNormalsPtr normals1, normals2;
  {
    pcl::ScopeTime t("descriptors computation");
    normals1 = computeSurfaceNormals(cloud1, normal_radius);
    descriptors1 = computeLocalDescriptors(cloud1, normals1, keypoints1,
                                           Descriptor::PFH, descriptor_radius);
    normals2 = computeSurfaceNormals(cloud2, normal_radius);
    descriptors2 = computeLocalDescriptors(cloud2, normals2, keypoints2,
                                           Descriptor::PFH, descriptor_radius);
  }

  std::cout << "extracted descriptors:" << std::endl;
  printPointCloud2Summary(*descriptors1);

  visualiseNormals(cloud1, normals1);

  /* compute correspondences */
  CorrespondencesPtr inliers;
  Eigen::Matrix4f transform;

  {
    pcl::ScopeTime t("finding correspondences");
    CorrespondencesPtr correspondences =
        findFeatureCorrespondences(descriptors1, descriptors2);
    transform = estimateTransformFromCorrespondences(
        keypoints1, keypoints2, correspondences, inliers, inlier_threshold);
  }

  visualiseCorrespondences(cloud1, keypoints1, cloud2, keypoints2, inliers);

  // Transform the source point to align them with the target points
  PointCloudPtr cloud1_aligned(new PointCloud);
  pcl::transformPointCloud(*cloud1, *cloud1_aligned, transform);
  visualisePointClouds(cloud1_aligned, cloud2);

  {
    pcl::ScopeTime t("initial alignment");
    transform = estimateTransformFromDescriptorsSets(
        keypoints1, descriptors1, keypoints2, descriptors2, inlier_threshold,
        max_correspondence_distance, nr_iterations);
  }

  // Transform the source point to align them with the target points
  pcl::transformPointCloud(*cloud1, *cloud1_aligned, transform);
  visualisePointClouds(cloud1_aligned, cloud2);

  PointCloudPtr aligned(new PointCloud);
  {
    pcl::ScopeTime t("ICP alignment");
    estimateTransformICP(cloud1_aligned, cloud2, aligned, max_iterations,
                         inlier_threshold);
  }
  visualisePointClouds(cloud1_aligned, aligned);

  return 0;
}
