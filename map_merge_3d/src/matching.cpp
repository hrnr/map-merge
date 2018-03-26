#include <map_merge_3d/matching.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>

// matches reciprocal correspondences among k-nearest matches
CorrespondencesPtr
findFeatureCorrespondences(const LocalDescriptorsPtr &source_descriptors,
                           const LocalDescriptorsPtr &target_descriptors)
{
  CorrespondencesPtr result(new Correspondences);
  result->reserve(source_descriptors->size());

  // Use a KdTree to search for the nearest matches in feature space
  pcl::KdTreeFLANN<LocalDescriptorT> target_search;
  target_search.setInputCloud(target_descriptors);
  target_search.setSortedResults(true);

  pcl::KdTreeFLANN<LocalDescriptorT> source_search;
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
