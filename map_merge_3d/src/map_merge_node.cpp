#include <map_merge_3d/map_merge_node.h>

#include <pcl_ros/point_cloud.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_eigen/tf2_eigen.h>

namespace map_merge_3d
{
MapMerge3d::MapMerge3d() : subscriptions_size_(0)
{
  ros::NodeHandle private_nh("~");
  std::string merged_map_topic;
  bool publish_tf = true;

  private_nh.param("compositing_rate", compositing_rate_, 0.3);
  private_nh.param("discovery_rate", discovery_rate_, 0.05);
  private_nh.param("estimation_rate", estimation_rate_, 0.01);
  private_nh.param<std::string>("robot_map_topic", robot_map_topic_, "map");
  private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
  private_nh.param<std::string>("merged_map_topic", merged_map_topic, "map");
  private_nh.param<std::string>("world_frame", world_frame_, "world");
  private_nh.param("publish_tf", publish_tf, true);
  // registration parameters
  map_merge_params_ = MapMergingParams::fromROSNode(private_nh);

  /* publishing */
  merged_map_publisher_ =
      node_.advertise<PointCloud>(merged_map_topic, 50, true);

  /* periodical discovery, estimation, compositing */
  compositing_timer_ =
      node_.createTimer(ros::Duration(1. / compositing_rate_),
                        [this](const ros::TimerEvent&) { mapCompositing(); });
  discovery_timer_ =
      node_.createTimer(ros::Duration(1. / discovery_rate_),
                        [this](const ros::TimerEvent&) { discovery(); });
  estimation_timer_ = node_.createTimer(
      ros::Duration(1. / estimation_rate_),
      [this](const ros::TimerEvent&) { transformsEstimation(); });

  // tf needs to publish at constant frequency to avoid transform lookup fails
  if (publish_tf) {
    tf_thread_ = std::thread([this]() {
      ros::Rate rate(30.0);
      while (node_.ok()) {
        publishTF();
        rate.sleep();
      }
    });
  }
}

/*
 * Dynamic robots discovery
 */
void MapMerge3d::discovery()
{
  ROS_DEBUG("Robot discovery started.");

  ros::master::V_TopicInfo topic_infos;
  std::string robot_name;
  std::string map_topic;

  ros::master::getTopics(topic_infos);

  for (const auto& topic : topic_infos) {
    // we check only map topic
    if (!isRobotMapTopic(topic)) {
      continue;
    }

    robot_name = robotNameFromTopic(topic.name);
    if (robots_.count(robot_name)) {
      // we already know this robot
      continue;
    }

    ROS_INFO("adding robot [%s] to system", robot_name.c_str());
    {
      std::lock_guard<std::mutex> lock(subscriptions_mutex_);
      subscriptions_.emplace_front();
      ++subscriptions_size_;
    }

    // no locking here. robots_ are used only in this procedure
    MapSubscription& subscription = subscriptions_.front();
    robots_.insert({robot_name, &subscription});

    /* subscribe callbacks */
    map_topic = ros::names::append(robot_name, robot_map_topic_);
    ROS_INFO("Subscribing to MAP topic: %s.", map_topic.c_str());
    subscription.map_sub = node_.subscribe<PointCloud>(
        map_topic, 50, [this, &subscription](const PointCloudConstPtr& msg) {
          mapUpdate(msg, subscription);
        });
  }

  ROS_DEBUG("Robot discovery finished.");
}

/*
 * Composing maps according to computed transforms
 */
void MapMerge3d::mapCompositing()
{
  ROS_DEBUG("Map compositing started.");

  std::vector<PointCloudConstPtr> clouds = getMaps();
  if (clouds.empty()) {
    return;
  }
  std::vector<Eigen::Matrix4f> transforms = getTransforms();
  // shrink clouds if we have subscribed some new maps since the last
  // estimation
  clouds.resize(transforms.size());

  PointCloudPtr merged_map =
      composeMaps(clouds, transforms, map_merge_params_.output_resolution);
  if (!merged_map) {
    return;
  }

  std_msgs::Header header;
  header.frame_id = world_frame_;
  header.stamp = ros::Time::now();
  pcl_conversions::toPCL(header, merged_map->header);
  merged_map_publisher_.publish(merged_map);

  ROS_DEBUG("Map compositing finished.");
}

void MapMerge3d::transformsEstimation()
{
  ROS_DEBUG("Transform estimation started.");
  std::vector<PointCloudConstPtr> clouds = getMaps();
  if (clouds.empty()) {
    return;
  }

  std::vector<Eigen::Matrix4f> transforms =
      estimateMapsTransforms(clouds, map_merge_params_);

  // set transforms thread-safe
  {
    std::lock_guard<std::mutex> lock(transforms_mutex_);
    transforms_ = transforms;
  }
  // notify tf publisher that transforms changed
  tf_current_flag_.clear();

  ROS_DEBUG("Transform estimation finished.");
}

void MapMerge3d::mapUpdate(const PointCloud::ConstPtr& msg,
                           MapSubscription& subscription)
{
  ROS_DEBUG("received map update");
  std::lock_guard<std::mutex> lock(subscription.mutex);

  subscription.map = msg;
}

std::vector<PointCloudConstPtr> MapMerge3d::getMaps()
{
  std::vector<PointCloudConstPtr> clouds;
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
  clouds.reserve(subscriptions_size_);
  for (auto& subscription : subscriptions_) {
    std::lock_guard<std::mutex> lock2(subscription.mutex);
    clouds.emplace_back(subscription.map);
  }

  return clouds;
}

std::vector<Eigen::Matrix4f> MapMerge3d::getTransforms()
{
  std::lock_guard<std::mutex> lock(transforms_mutex_);
  return transforms_;
}

std::string MapMerge3d::robotNameFromTopic(const std::string& topic)
{
  return ros::names::parentNamespace(topic);
}

/* identifies topic via suffix */
bool MapMerge3d::isRobotMapTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name);
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_map_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.name.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  /* we support only PointCloud2 as maps */
  bool is_occupancy_grid = topic.datatype == "sensor_msgs/PointCloud2";

  /* we don't want to subcribe on published merged map */
  bool is_our_topic = merged_map_publisher_.getTopic() == topic.name;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

static inline std::vector<geometry_msgs::TransformStamped> computeTFTransforms(
    const std::vector<Eigen::Matrix4f>& transforms,
    const std::vector<PointCloudConstPtr>& maps, const std::string& world_frame)
{
  std::vector<geometry_msgs::TransformStamped> result;
  result.reserve(transforms.size());
  // convert to ROS transforms
  for (const auto& transform : transforms) {
    Eigen::Affine3d affine;
    affine.matrix() = transform.cast<double>();
    result.emplace_back(tf2::eigenToTransform(affine));
  }
  // fill frame_ids
  for (size_t i = 0; i < result.size(); ++i) {
    result[i].header.frame_id = world_frame;
    result[i].child_frame_id = maps[i]->header.frame_id;
  }

  return result;
}

void MapMerge3d::publishTF()
{
  if (!tf_current_flag_.test_and_set()) {
    // need to recalculate stored transforms
    auto transforms = getTransforms();  // this is thread-safe access
    auto maps = getMaps();
    tf_transforms_ = computeTFTransforms(transforms, maps, world_frame_);
  }
  if (tf_transforms_.empty()) {
    return;
  }

  // no locking. tf_transforms_ is always accessed only from tf thread
  ros::Time now = ros::Time::now();
  for (auto& transform : tf_transforms_) {
    transform.header.stamp = now;
  }
  tf_publisher_.sendTransform(tf_transforms_);
}

}  // namespace map_merge_3d

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_merge");
  // this package is still in development -- start wil debugging enabled
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  map_merge_3d::MapMerge3d map_merge_node;
  // use all threads for spinning
  ros::MultiThreadedSpinner spinner;
  spinner.spin();
  return 0;
}
