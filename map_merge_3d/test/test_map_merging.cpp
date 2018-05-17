#include <gtest/gtest.h>
#include <ros/ros.h>

#include <map_merge_3d/map_merging.h>

using Eigen::Matrix4f;
using namespace map_merge_3d;

TEST(estimateMapsTransforms, empty)
{
  std::vector<Matrix4f> result = estimateMapsTransforms({}, MapMergingParams());
  EXPECT_TRUE(result.empty());
}

TEST(estimateMapsTransforms, one)
{
  std::vector<Matrix4f> result = estimateMapsTransforms(
      {PointCloudConstPtr(new PointCloud)}, MapMergingParams());
  EXPECT_EQ(result.size(), 1);
  EXPECT_EQ(result[0], Matrix4f::Identity());
}

TEST(composeMaps, empty)
{
  PointCloudPtr result = composeMaps({}, {}, 0.0);
  EXPECT_EQ(result, nullptr);
}

TEST(composeMaps, wrongSizes)
{
  EXPECT_ANY_THROW(composeMaps({nullptr}, {}, 0.0));
}

TEST(composeMaps, one)
{
  PointCloudPtr result = composeMaps({PointCloudConstPtr(new PointCloud)},
                                     {Matrix4f::Identity()}, 0.0);
  EXPECT_NE(result, nullptr);
  EXPECT_EQ(result->size(), 0);
}

int main(int argc, char** argv)
{
  ros::Time::init();
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
