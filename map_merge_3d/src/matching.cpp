#include <map_merge_3d/matching.h>

#include <pcl/conversions.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/kdtree.h>

// matches reciprocal correspondences among k-nearest matches
template <typename DescriptorT>
CorrespondencesPtr
findFeatureCorrespondences(const LocalDescriptorsPtr &source_descriptors_,
                           const LocalDescriptorsPtr &target_descriptors_)
{
  typedef pcl::PointCloud<DescriptorT> DescriptorsPointCLoud1;

  typename DescriptorsPointCLoud1::Ptr source_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*source_descriptors_, *source_descriptors);
  typename DescriptorsPointCLoud1::Ptr target_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*target_descriptors_, *target_descriptors);

  CorrespondencesPtr result(new Correspondences);
  result->reserve(source_descriptors->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<DescriptorT> target_search;
  target_search.setInputCloud(target_descriptors);
  target_search.setSortedResults(true);

  pcl::search::KdTree<DescriptorT> source_search;
  source_search.setInputCloud(source_descriptors);
  source_search.setSortedResults(true);

  // storing nearest k search result
  const int k = 5;
  // forward search results
  std::vector<int> k_indices(k);
  std::vector<float> k_squared_distances(k);
  // backward search results
  std::vector<int> k_indices_back(k);
  std::vector<float> k_squared_distances_back(k);

  for (size_t i = 0; i < source_descriptors->size(); ++i) {
    // source to target match
    target_search.nearestKSearch(*source_descriptors, int(i), k, k_indices,
                                 k_squared_distances);
    // try to cross-match all k source to target matches
    for (size_t j = 0; j < k; ++j) {
      int match = k_indices[j];
      float dist = k_squared_distances[j];
      // target to source match
      source_search.nearestKSearch(*target_descriptors, match, k,
                                   k_indices_back, k_squared_distances_back);
      // seach for original index in back-matches
      for (int back_i : k_indices_back) {
        if (back_i == int(i)) {
          // we have cross match
          result->emplace_back(i, match, dist);

          /* we stop here. pcl::SampleConsensusModelRegistration can not handle
           * multiple matches per point. The current match should already be the
           * best as the results are sorted. */
          j = k;  // stop back matching
          break;
        }
      }
    }
  }
  std::cout << "findFeatureCorrespondences cross-matches: " << result->size()
            << std::endl;

  return result;
}

// matches reciprocal correspondences among k-nearest matches
CorrespondencesPtr
findFeatureCorrespondences(const LocalDescriptorsPtr &source_descriptors,
                           const LocalDescriptorsPtr &target_descriptors)
{
  if (source_descriptors->fields.size() != 1 ||
      target_descriptors->fields.size() != 1) {
    throw std::runtime_error("findFeatureCorrespondences: descriptors must "
                             "contain exactly one field with descritors.");
  }

  auto name = source_descriptors->fields[0].name;

  if (name == "pfh") {
    return findFeatureCorrespondences<pcl::PFHSignature125>(source_descriptors,
                                                            target_descriptors);
  }

  throw std::runtime_error("findFeatureCorrespondences: unknown descriptor "
                           "type.");
}

Eigen::Matrix4f estimateTransformFromCorrespondences(
    const PointCloudPtr &source_keypoints,
    const PointCloudPtr &target_keypoints,
    const CorrespondencesPtr &correspondences, CorrespondencesPtr &inliers,
    double inlier_threshold)
{
  Eigen::Matrix4f result;
  inliers.reset(new Correspondences);

  pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> ransac;
  ransac.setInputSource(source_keypoints);
  ransac.setInputTarget(target_keypoints);
  ransac.setInputCorrespondences(correspondences);
  ransac.setInlierThreshold(inlier_threshold);
  ransac.getCorrespondences(*inliers);

  std::cout << "estimateTransformFromCorrespondences inlier threshold: "
            << ransac.getInlierThreshold() << std::endl;
  std::cout << "estimateTransformFromCorrespondences ransac transform: "
            << std::endl
            << ransac.getBestTransformation() << std::endl;

  // check if we succeded to find a model. unfortunately there is not a better
  // way.
  if (ransac.getBestTransformation().isIdentity()) {
    // ransac failed to find a resonable model
    std::cout << "estimateTransformFromCorrespondences ransac failed"
              << std::endl;
    result.setZero();
    inliers->clear();  // ransac will set this to original matches
    return result;
  }

  pcl::registration::TransformationEstimationSVD<PointT, PointT> svd;
  svd.estimateRigidTransformation(*source_keypoints, *target_keypoints,
                                  *inliers, result);

  std::cout << "estimateTransformFromCorrespondences inliers: "
            << inliers->size() << std::endl;

  return result;
}

template <typename DescriptorT>
Eigen::Matrix4f estimateTransformFromDescriptorsSets(
    const PointCloudPtr &source_keypoints,
    const LocalDescriptorsPtr &source_descriptors_,
    const PointCloudPtr &target_keypoints,
    const LocalDescriptorsPtr &target_descriptors_, double min_sample_distance,
    double max_correspondence_distance, int max_iterations)
{
  // convert to required PointCloudType
  typedef pcl::PointCloud<DescriptorT> DescriptorsPointCLoud1;
  typename DescriptorsPointCLoud1::Ptr source_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*source_descriptors_, *source_descriptors);
  typename DescriptorsPointCLoud1::Ptr target_descriptors(
      new DescriptorsPointCLoud1);
  pcl::fromPCLPointCloud2(*target_descriptors_, *target_descriptors);

  pcl::SampleConsensusInitialAlignment<PointT, PointT, DescriptorT> estimator;
  estimator.setMinSampleDistance(min_sample_distance);
  estimator.setMaxCorrespondenceDistance(max_correspondence_distance);
  estimator.setMaximumIterations(max_iterations);

  estimator.setInputSource(source_keypoints);
  estimator.setSourceFeatures(source_descriptors);

  estimator.setInputTarget(target_keypoints);
  estimator.setTargetFeatures(target_descriptors);

  PointCloud registration_output;
  estimator.align(registration_output);

  std::cout << "initial alignment converged:" << estimator.hasConverged()
            << std::endl;
  std::cout << "initial alignment score:" << estimator.getFitnessScore()
            << std::endl;

  return (estimator.getFinalTransformation());
}

Eigen::Matrix4f estimateTransformFromDescriptorsSets(
    const PointCloudPtr &source_keypoints,
    const LocalDescriptorsPtr &source_descriptors,
    const PointCloudPtr &target_keypoints,
    const LocalDescriptorsPtr &target_descriptors, double min_sample_distance,
    double max_correspondence_distance, int max_iterations)
{
  if (source_descriptors->fields.size() != 1 ||
      target_descriptors->fields.size() != 1) {
    throw std::runtime_error("estimateTransformFromDescriptorsSets: "
                             "descriptors must contain exactly one field with "
                             "descritors.");
  }

  auto name = source_descriptors->fields[0].name;

  if (name == "pfh") {
    return estimateTransformFromDescriptorsSets<pcl::PFHSignature125>(
        source_keypoints, source_descriptors, target_keypoints,
        target_descriptors, min_sample_distance, max_correspondence_distance,
        max_iterations);
  }

  throw std::runtime_error("estimateTransformFromDescriptorsSets: unknown "
                           "descriptor type.");
}
