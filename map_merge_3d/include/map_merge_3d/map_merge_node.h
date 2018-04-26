#ifndef MAP_MERGE_MAP_MERGE_NODE_H_
#define MAP_MERGE_MAP_MERGE_NODE_H_

#include <atomic>
#include <forward_list>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <map_merge_3d/map_merging.h>
#include <map_merge_3d/typedefs.h>

namespace map_merge_3d
{
/**
 * @defgroup node ROS node
 * @brief ROS interface.
 * @details Manages maps discovery, transforms estimation and the global
 * map publishing.
 * @{
 */

/**
 * @brief ROS node class.
 * @details Runs robot discovery, transforms estimation and map compositing at
 * predefined rates (from ROS parameters). Does not spin on its own.
 *
 */
class MapMerge3d
{
private:
  struct MapSubscription {
    // protects map
    std::mutex mutex;
    PointCloudConstPtr map;
    ros::Subscriber map_sub;
  };

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
  // periodical callbacks
  ros::Timer compositing_timer_;
  ros::Timer discovery_timer_;
  ros::Timer estimation_timer_;
  // transforms for tf
  std::vector<geometry_msgs::TransformStamped> tf_transforms_;
  tf2_ros::TransformBroadcaster tf_publisher_;
  std::thread tf_thread_;             //  tf needs it own thread
  std::atomic_flag tf_current_flag_;  // whether tf_transforms_ are up to date
                                      // with transforms_

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
  void publishTF();

public:
  MapMerge3d();

  /**
   * @brief Initiates discovery of new robots (maps) under current ROS core.
   * @details When new maps topics are found, there are added for merging. This
   * function is thread-safe
   */
  void discovery();

  /**
   * @brief Composes and publishes the global map based on estimated
   * transformations
   * @details This function is thread-safe
   */
  void mapCompositing();

  /**
   * @brief Estimates transformations between maps
   * @details This function is thread-safe
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
   * transform between map and global reference frame.
   * @details This function is thread-safe
   * @return all currently estimated transforms. Zero Matrix if transform could
   * not be estimated.
   */
  std::vector<Eigen::Matrix4f> getTransforms();
};

///@} group node

}  // namespace map_merge_3d

#endif  // MAP_MERGE_MAP_MERGE_NODE_H_
