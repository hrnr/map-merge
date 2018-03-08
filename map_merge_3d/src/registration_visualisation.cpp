#include <map_merge_3d/typedefs.h>

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

void displayMatch(PointCloudPtr cloud1, PointCloudPtr cloud2) {
  std::cout << "displaying pointclouds of sizes: " << cloud1->size() << ", "
            << cloud2->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointclouds match");
  vis.addPointCloud(cloud1, ColorHandlerT(cloud1, 0.0, 255.0, 0.0), "cloud1");
  vis.addPointCloud(cloud2, ColorHandlerT(cloud2, 0.0, 0.0, 255.0), "cloud2");
  vis.spin();
  vis.close();
}

void downSample(PointCloudPtr input, PointCloudPtr output, float leaf_size) {
  pcl::VoxelGrid<PointT> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(input);
  filter.filter(*output);
}

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
                                float max_iterations) {

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

/* Use SIFTKeypoint to detect a set of keypoints
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   min_scale
 *     The smallest scale in the difference-of-Gaussians (DoG) scale-space
 *   nr_octaves
 *     The number of times the scale doubles in the DoG scale-space
 *   nr_scales_per_octave
 *     The number of scales computed for each doubling
 *   min_contrast
 *     The minimum local contrast that must be present for a keypoint to be
 * detected
 * Return: A pointer to a point cloud of keypoints
 */
PointCloudPtr detectKeypoints(const PointCloudPtr &points, float min_scale,
                              int nr_octaves, int nr_scales_per_octave,
                              float min_contrast) {
  pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detect;
  sift_detect.setSearchMethod(
      pcl::search::Search<PointT>::Ptr(new pcl::search::KdTree<PointT>));
  sift_detect.setScales(min_scale, nr_octaves, nr_scales_per_octave);
  sift_detect.setMinimumContrast(min_contrast);
  sift_detect.setInputCloud(points);

  pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
  sift_detect.compute(keypoints_temp);

  PointCloudPtr keypoints(new PointCloud);
  pcl::copyPointCloud(keypoints_temp, *keypoints);

  return (keypoints);
}

/* Use FPFHEstimation to compute local feature descriptors around each keypoint
 * Inputs:
 *   points
 *     The input point cloud
 *   normals
 *     The input surface normals
 *   keypoints
 *     A cloud of keypoints specifying the positions at which the descriptors
 * should be computed
 *   feature_radius
 *     The size of the neighborhood from which the local descriptors will be
 * computed
 * Return: A pointer to a LocalDescriptors (a cloud of LocalDescriptorT points)
 */
LocalDescriptorsPtr computeLocalDescriptors(const PointCloudPtr &points,
                                            const SurfaceNormalsPtr &normals,
                                            const PointCloudPtr &keypoints,
                                            float feature_radius) {
  pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
  fpfh_estimation.setSearchMethod(
      pcl::search::Search<PointT>::Ptr(new pcl::search::KdTree<PointT>));
  fpfh_estimation.setRadiusSearch(feature_radius);
  fpfh_estimation.setSearchSurface(points);
  fpfh_estimation.setInputNormals(normals);
  fpfh_estimation.setInputCloud(keypoints);
  LocalDescriptorsPtr local_descriptors(new LocalDescriptors);
  fpfh_estimation.compute(*local_descriptors);

  return (local_descriptors);
}

/* Use NormalEstimation to estimate a cloud's surface normals
 * Inputs:
 *   input
 *     The input point cloud
 *   radius
 *     The size of the local neighborhood used to estimate the surface
 * Return: A pointer to a SurfaceNormals point cloud
 */
SurfaceNormalsPtr computeSurfaceNormals(const PointCloudPtr &input,
                                        float radius) {
  pcl::NormalEstimation<PointT, NormalT> normal_estimation;
  normal_estimation.setSearchMethod(
      pcl::search::Search<PointT>::Ptr(new pcl::search::KdTree<PointT>));
  normal_estimation.setRadiusSearch(radius);
  normal_estimation.setInputCloud(input);
  SurfaceNormalsPtr normals(new SurfaceNormals);
  normal_estimation.compute(*normals);

  return (normals);
}

void findFeatureCorrespondences(const LocalDescriptorsPtr &source_descriptors,
                                const LocalDescriptorsPtr &target_descriptors,
                                std::vector<int> &correspondences_out,
                                std::vector<float> &correspondence_scores_out) {
  // Resize the output vector
  correspondences_out.resize(source_descriptors->size());
  correspondence_scores_out.resize(source_descriptors->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<LocalDescriptorT> target_search;
  target_search.setInputCloud(target_descriptors);

  pcl::KdTreeFLANN<LocalDescriptorT> source_search;
  source_search.setInputCloud(source_descriptors);

  // Find the index of the best match for each keypoint, and store it in
  // "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices(k);
  std::vector<float> k_squared_distances(k);
  for (size_t i = 0; i < source_descriptors->size(); ++i) {
    target_search.nearestKSearch(*source_descriptors, i, k, k_indices,
                                 k_squared_distances);
    int match = k_indices[0];
    float dist = k_squared_distances[0];
    // search the same in source
    source_search.nearestKSearch(*target_descriptors, match, k, k_indices,
                                 k_squared_distances);
    if (k_indices[0] == i) {
      // we have cross match
      correspondences_out[i] = match;
      correspondence_scores_out[i] = dist;
    } else {
      correspondences_out[i] = i;
      correspondence_scores_out[i] = std::numeric_limits<float>::infinity();
    }
  }
}

/* Use SampleConsensusInitialAlignment to find a rough alignment from the source
 * cloud to the target cloud by finding
 * correspondences between two sets of local features
 * Inputs:
 *   source_points
 *     The "source" points, i.e., the points that must be transformed to align
 * with the target point cloud
 *   source_descriptors
 *     The local descriptors for each source point
 *   target_points
 *     The "target" points, i.e., the points to which the source point cloud
 * will be aligned
 *   target_descriptors
 *     The local descriptors for each target point
 *   min_sample_distance
 *     The minimum distance between any two random samples
 *   max_correspondence_distance
 *     The
 *   nr_interations
 *     The number of RANSAC iterations to perform
 * Return: A transformation matrix that will roughly align the points in source
 * to the points in target
 */
Eigen::Matrix4f
computeInitialAlignment(const PointCloudPtr &source_points,
                        const LocalDescriptorsPtr &source_descriptors,
                        const PointCloudPtr &target_points,
                        const LocalDescriptorsPtr &target_descriptors,
                        float min_sample_distance,
                        float max_correspondence_distance, int nr_iterations) {
  pcl::SampleConsensusInitialAlignment<PointT, PointT, LocalDescriptorT> sac_ia;
  // sac_ia.setMinSampleDistance(min_sample_distance);
  // sac_ia.setMaxCorrespondenceDistance(max_correspondence_distance);
  sac_ia.setMaximumIterations(nr_iterations);

  sac_ia.setInputCloud(source_points);
  sac_ia.setSourceFeatures(source_descriptors);

  sac_ia.setInputTarget(target_points);
  sac_ia.setTargetFeatures(target_descriptors);

  PointCloudPtr registration_output(new PointCloud);
  sac_ia.align(*registration_output);

  displayMatch(target_points, registration_output);

  return (sac_ia.getFinalTransformation());
}

void visualizeCorrespondences(const PointCloudPtr points1,
                              const PointCloudPtr keypoints1,
                              const PointCloudPtr points2,
                              const PointCloudPtr keypoints2,
                              const std::vector<int> &correspondences,
                              const std::vector<float> &correspondence_scores,
                              int max_to_display) {
  // We want to visualize two clouds side-by-side, so do to this, we'll make
  // copies of the clouds and transform them
  // by shifting one to the left and the other to the right.  Then we'll draw
  // lines between the corresponding points

  // Create some new point clouds to hold our transformed data
  PointCloudPtr points_left(new PointCloud);
  PointCloudPtr keypoints_left(new PointCloud);
  PointCloudPtr points_right(new PointCloud);
  PointCloudPtr keypoints_right(new PointCloud);

  // Shift the first clouds' points to the left
  // const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate(0.4, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation(0, 0, 0, 0);
  pcl::transformPointCloud(*points1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud(*keypoints1, *keypoints_left, -translate,
                           no_rotation);

  // Shift the second clouds' points to the right
  pcl::transformPointCloud(*points2, *points_right, translate, no_rotation);
  pcl::transformPointCloud(*keypoints2, *keypoints_right, translate,
                           no_rotation);

  // Add the clouds to the visualizer
  pcl::visualization::PCLVisualizer vis;
  ColorHandlerT green(points_left, 0.0, 255.0, 0.0);
  vis.addPointCloud(points_left, green, "points_left");
  ColorHandlerT blue(points_right, 0.0, 0.0, 255.0);
  vis.addPointCloud(points_right, blue, "points_right");

  // Compute the weakest correspondence score to display
  std::vector<float> temp(correspondence_scores);
  std::sort(temp.begin(), temp.end());
  if (max_to_display >= temp.size())
    max_to_display = temp.size() - 1;
  float threshold = temp[max_to_display];

  std::cout << max_to_display << std::endl;

  // Draw lines between the best corresponding points
  for (size_t i = 0; i < keypoints_left->size(); ++i) {
    if (correspondence_scores[i] > threshold) {
      continue; // Don't draw weak correspondences
    }

    // Get the pair of points
    const PointT &p_left = keypoints_left->points[i];
    const PointT &p_right = keypoints_right->points[correspondences[i]];

    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max(r, std::max(g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;

    // Generate a unique string for each line
    std::stringstream ss("line");
    ss << i;

    // Draw the line
    vis.addLine(p_left, p_right, r, g, b, ss.str());
  }

  vis.resetCamera();
  vis.spin();
}

void estimateTransformICP(PointCloudPtr cloud1, PointCloudPtr cloud2,
                          PointCloudPtr output = nullptr, float max_iterations = 500,
                          float max_correspondence_distance = 0.05) {
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setInputTarget(cloud1);
  icp.setInputSource(cloud2);
  // icp.setMaxCorrespondenceDistance(max_correspondence_distance);
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
              << "The score is " << icp.getFitnessScore();
  } else {
    std::cout << "ICP did not converge.";
  }
}

void displayCloud(PointCloudPtr cloud) {
  std::cout << "displaying pointcloud of size: " << cloud->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointcloud");

  vis.addPointCloud(cloud, ColorHandlerT(cloud, 0.0, 255.0, 0.0), "cloud");
  vis.spin();
  vis.close();
  vis.spinOnce();
}

void displayNormals(PointCloudPtr cloud, SurfaceNormalsPtr normals) {
  std::cout << "displaying pointcloud of size: " << cloud->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointcloud and normals");

  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");
  vis.addPointCloudNormals<PointT, NormalT>(cloud, normals, 5, 0.1, "normals");

  vis.spin();
  vis.close();
  vis.spinOnce();
}

void displayKeypoints(PointCloudPtr cloud, PointCloudPtr keypoints) {
  std::cout << "pointcloud size: " << cloud->size() << std::endl
            << "keypoints: " << keypoints->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointcloud keypoints");

  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");

  ColorHandlerT red(keypoints, 255, 0, 0);
  vis.addPointCloud(keypoints, red, "keypoints");
  vis.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

  vis.spin();
  vis.close();
  vis.spinOnce();
}

int main(int argc, char **argv) {
  PointCloudPtr cloud1(new PointCloud);
  PointCloudPtr cloud2(new PointCloud);
  PointCloudPtr cloud1_orig(new PointCloud);
  PointCloudPtr cloud2_orig(new PointCloud);
  // Get input object and scene
  // if (argc >= 3) {
  //   pcl::console::print_error("Syntax is: %s object.pcd scene.pcd\n",
  //   argv[0]);
  //   return (1);
  // }

  // Load object and scene
  pcl::console::print_highlight("Loading point clouds...\n");
  if (pcl::io::loadPCDFile<PointT>(argv[1], *cloud1) < 0 ||
      pcl::io::loadPCDFile<PointT>(argv[2], *cloud2) < 0) {
    pcl::console::print_error("Error loading object/scene file!\n");
    return (1);
  }

  float leaf_size = 0.1f;
  pcl::console::parse_argument(argc, argv, "--resolution", leaf_size);

  downSample(cloud1, cloud1, leaf_size);
  downSample(cloud2, cloud2, leaf_size);

  // displayMatch(cloud1, cloud2);

  // PointCloudPtr aligned(new PointCloud);
  // estimateTransformICP(cloud1, cloud2, aligned);

  // displayMatch(cloud1, aligned);
  const float min_scale = 0.01;
  const int nr_octaves = 3;
  const int nr_octaves_per_scale = 3;
  // const float min_contrast = 10.0;
  const float min_contrast = 15.0;

  PointCloudPtr keypoints1, keypoints2;
  {
    pcl::ScopeTime t("feature detection");
    keypoints1 = detectKeypoints(cloud1, min_scale, nr_octaves,
                                 nr_octaves_per_scale, min_contrast);
  }
  keypoints2 = detectKeypoints(cloud2, min_scale, nr_octaves,
                               nr_octaves_per_scale, min_contrast);

  float normal_radius = 0.03;
  pcl::console::parse_argument(argc, argv, "--normal_radius", normal_radius);
  float descriptor_radius = 0.08;
  pcl::console::parse_argument(argc, argv, "--descriptor_radius",
                               descriptor_radius);

  LocalDescriptorsPtr descriptors1, descriptors2;
  SurfaceNormalsPtr normals1, normals2;
  {
    pcl::ScopeTime t("descriptors computation");
    normals1 = computeSurfaceNormals(cloud1, normal_radius);
    descriptors1 = computeLocalDescriptors(cloud1, normals1, keypoints1,
                                           descriptor_radius);
  }
  normals2 = computeSurfaceNormals(cloud2, normal_radius);
  descriptors2 =
      computeLocalDescriptors(cloud2, normals2, keypoints2, descriptor_radius);

  displayKeypoints(cloud1, keypoints1);
  // displayKeypoints(cloud2, keypoints2);

  displayNormals(cloud1, normals1);

  // std::vector<int> correspondences;
  // std::vector<float> correspondence_scores;
  // findFeatureCorrespondences(descriptors1, descriptors2, correspondences,
  //                            correspondence_scores);

  // visualizeCorrespondences(cloud1, keypoints1, cloud2, keypoints2,
  //                          correspondences, correspondence_scores, 0);

  const float min_sample_distance = 0.5;
  // const float min_sample_distance = 0.025;
  float max_correspondence_distance = 0.2;
  // float max_correspondence_distance = 0.01;
  const int nr_iterations = 500;
  Eigen::Matrix4f transform = computeInitialAlignment(
      keypoints1, descriptors1, keypoints2, descriptors2, min_sample_distance,
      max_correspondence_distance, nr_iterations);

  // Transform the source point to align them with the target points
  PointCloudPtr cloud1_aligned(new PointCloud);
  pcl::transformPointCloud(*cloud1, *cloud1_aligned, transform);
  displayMatch(cloud1_aligned, cloud2);

  max_correspondence_distance = 0.05;
  float outlier_rejection_threshold = 0.05;
  float transformation_epsilon = 0;
  int max_iterations = 200;

  // transform = refineAlignment(
  //     cloud1, cloud2, transform, max_correspondence_distance,
  //     outlier_rejection_threshold, transformation_epsilon, max_iterations);
  // pcl::transformPointCloud(*cloud1, *cloud1_aligned, transform);
  // displayMatch(cloud1_aligned, cloud2);

  PointCloudPtr aligned(new PointCloud);
  estimateTransformICP(cloud1_aligned, cloud2, aligned, max_iterations, max_correspondence_distance);
  displayMatch(cloud1_aligned, aligned);

  return 0;
}
