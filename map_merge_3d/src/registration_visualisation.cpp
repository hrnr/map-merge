#include <map_merge_3d/typedefs.h>

#include <map_merge_3d/features.h>
#include <map_merge_3d/map_merging.h>
#include <map_merge_3d/matching.h>
#include "visualise.h"

#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

using namespace map_merge_3d;

static inline void printPointCloud2Summary(const pcl::PCLPointCloud2 &v)
{
  for (size_t i = 0; i < v.fields.size(); ++i) {
    std::cout << "fields[" << i << "]:" << std::endl;
    std::cout << v.fields[i] << std::endl;
  }
}

int main(int argc, char **argv)
{
  std::vector<int> pcd_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  if (pcd_file_indices.size() != 2) {
    pcl::console::print_error("Need exactly 2 input files!\n");
    return -1;
  }

  const MapMergingParams params = MapMergingParams::fromCommandLine(argc, argv);
  std::cout << "params: " << std::endl << params << std::endl;

  PointCloudPtr cloud1_full(new PointCloud);
  PointCloudPtr cloud2_full(new PointCloud);
  PointCloudPtr cloud1, cloud2;

  // load input pcd files
  if (pcl::io::loadPCDFile<PointT>(argv[pcd_file_indices[0]], *cloud1_full) <
          0 ||
      pcl::io::loadPCDFile<PointT>(argv[pcd_file_indices[1]], *cloud2_full) <
          0) {
    pcl::console::print_error("Error loading input file!\n");
    return -1;
  }
  std::cout << "loaded pointclouds of sizes: " << cloud1_full->size() << ", "
            << cloud2_full->size() << std::endl;

  pcl::console::print_highlight("Downsampling to working resolution.\n");
  {
    pcl::ScopeTime t("downsampling");
    cloud1 = downSample(cloud1_full, params.resolution);
    cloud2 = downSample(cloud2_full, params.resolution);
  }
  std::cout << "downsampled clouds to: " << cloud1->size() << ", "
            << cloud2->size() << std::endl;

  visualisePointCloud(cloud1);

  pcl::console::print_highlight("Removing outliers.\n");
  {
    pcl::ScopeTime t("removing outliers");
    cloud1 = removeOutliers(cloud1, params.descriptor_radius,
                            params.outliers_min_neighbours);
    cloud2 = removeOutliers(cloud2, params.descriptor_radius,
                            params.outliers_min_neighbours);
  }
  std::cout << "remaining points: " << cloud1->size() << ", " << cloud2->size()
            << std::endl;

  visualisePointCloud(cloud1);

  /* detect normals */
  pcl::console::print_highlight("Computing normals.\n");
  SurfaceNormalsPtr normals1, normals2;
  {
    pcl::ScopeTime t("normals computation");
    normals1 = computeSurfaceNormals(cloud1, params.normal_radius);
    normals2 = computeSurfaceNormals(cloud2, params.normal_radius);
  }

  /* detect keypoints */
  pcl::console::print_highlight("Detecting keypoints.\n");
  PointCloudPtr keypoints1, keypoints2;
  {
    pcl::ScopeTime t("keypoints detection");
    keypoints1 = detectKeypoints(cloud1, normals1, params.keypoint_type,
                                 params.keypoint_threshold,
                                 params.normal_radius, params.resolution);
    keypoints2 = detectKeypoints(cloud2, normals2, params.keypoint_type,
                                 params.keypoint_threshold,
                                 params.normal_radius, params.resolution);
  }
  std::cout << "keypoints count: " << keypoints1->size() << ", "
            << keypoints2->size() << std::endl;

  /* compute descriptors */
  pcl::console::print_highlight("Computing descriptors.\n");
  LocalDescriptorsPtr descriptors1, descriptors2;
  {
    pcl::ScopeTime t("descriptors computation");
    descriptors1 = computeLocalDescriptors(cloud1, normals1, keypoints1,
                                           params.descriptor_type,
                                           params.descriptor_radius);
    descriptors2 = computeLocalDescriptors(cloud2, normals2, keypoints2,
                                           params.descriptor_type,
                                           params.descriptor_radius);
  }

  std::cout << "extracted descriptors:" << std::endl;
  printPointCloud2Summary(*descriptors1);

  visualiseNormals(cloud1, normals1);
  visualiseKeypoints(cloud1, keypoints1);

  /* compute correspondences */
  pcl::console::print_highlight("Transform estimation using MATCHING.\n");
  CorrespondencesPtr correspondences, inliers;
  Eigen::Matrix4f transform;
  {
    pcl::ScopeTime t("finding correspondences");
    correspondences = findFeatureCorrespondences(descriptors1, descriptors2,
                                                 params.matching_k);
    transform = estimateTransformFromCorrespondences(keypoints1, keypoints2,
                                                     correspondences, inliers,
                                                     params.inlier_threshold);
  }

  std::cout << "cross-matches count: " << correspondences->size() << std::endl;
  std::cout << "inliers count: " << inliers->size() << std::endl;
  std::cout << "MATCHING est score: "
            << transformScore(cloud1_full, cloud2_full, transform,
                              params.max_correspondence_distance)
            << std::endl;

  visualiseCorrespondences(cloud1, keypoints1, cloud2, keypoints2, inliers);
  visualiseTransform(cloud1, cloud2, transform);

  pcl::console::print_highlight("Transform estimation using SAC_IA.\n");
  Eigen::Matrix4f transform_ia;
  {
    pcl::ScopeTime t("initial alignment");
    transform_ia = estimateTransformFromDescriptorsSets(
        keypoints1, descriptors1, keypoints2, descriptors2,
        params.inlier_threshold, params.max_correspondence_distance,
        params.max_iterations);
  }

  std::cout << "SAC_IA est score: "
            << transformScore(cloud1_full, cloud2_full, transform_ia,
                              params.max_correspondence_distance)
            << std::endl;

  visualiseTransform(cloud1, cloud2, transform_ia);

  pcl::console::print_highlight("Refining transform with ICP.\n");
  {
    pcl::ScopeTime t("ICP alignment");
    transform = estimateTransformICP(
        cloud1, cloud2, transform, params.max_correspondence_distance,
        params.inlier_threshold, params.max_iterations,
        params.transform_epsilon);
  }

  std::cout << "ICP est score: "
            << transformScore(cloud1_full, cloud2_full, transform,
                              params.max_correspondence_distance)
            << std::endl;
  std::cout << "final transformation: " << std::endl << transform << std::endl;

  visualiseTransform(cloud1, cloud2, transform);

  return 0;
}
