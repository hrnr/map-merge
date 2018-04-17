#ifndef MAP_MERGE_MAP_MERGE_NODE_H_
#define MAP_MERGE_MAP_MERGE_NODE_H_

#include <forward_list>
#include <mutex>
#include <unordered_map>

#include <ros/ros.h>

#include <map_merge_3d/map_merging.h>
#include <map_merge_3d/typedefs.h>

namespace map_merge_3d
{
struct MapSubscription {
  // protects map
  std::mutex mutex;
  PointCloudConstPtr map;
  ros::Subscriber map_sub;
};

class MapMerge3d
{
private:
  ros::NodeHandle node_;

  /* node parameters */
  double compositing_rate_;
  double discovery_rate_;
  double estimation_rate_;
  std::string robot_map_topic_;
  std::string robot_namespace_;
  std::string world_frame_;
  // compositing & estimation parameters
  MapMergingParams map_merge_params_;

  // publishing
  ros::Publisher merged_map_publisher_;
  // maps robots namespaces to maps. does not own
  std::unordered_map<std::string, MapSubscription*> robots_;
  // owns maps -- iterator safe
  std::forward_list<MapSubscription> subscriptions_;
  size_t subscriptions_size_;
  std::mutex subscriptions_mutex_;
  // estimated transforms between maps
  std::vector<Eigen::Matrix4f> transforms_;
  std::mutex transforms_mutex_;

  std::string robotNameFromTopic(const std::string& topic);
  bool isRobotMapTopic(const ros::master::TopicInfo& topic);

  void mapUpdate(const PointCloud::ConstPtr& msg,
                 MapSubscription& subscription);

  void execute(double rate, void (MapMerge3d::*function)());

public:
  MapMerge3d();

  void spin();

  void discovery();
  void mapCompositing();
  /**
   * @brief Estimates initial positions of maps
   */
  void transformsEstimation();

  /**
   * @brief Get currently stored maps
   * @details This function is thread-safe
   * @return all currently received maps
   */
  std::vector<PointCloudConstPtr> getMaps();

  /**
   * @brief Get currently stored transforms. For each map there should be a
   * trasform between map and global reference frame.
   * @details This function is thread-safe
   * @return all currently estimated transforms. Zero Matrix if transform could
   * not be estimated.
   */
  std::vector<Eigen::Matrix4f> getTransforms();
};

}  // namespace map_merge_3d

#endif  // MAP_MERGE_MAP_MERGE_NODE_H_
