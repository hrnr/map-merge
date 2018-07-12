#include "visualise.h"

#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#define VISUALISE 1

namespace map_merge_3d
{

#if VISUALISE

// use just one instance of vis to overcome bug PCL #172
static pcl::visualization::PCLVisualizer vis("cloud view");

static void show()
{
  vis.setBackgroundColor(255, 255, 255);
  vis.spin();
  vis.removeCorrespondences();
  vis.removeAllPointClouds();
  vis.removeAllShapes();
  vis.removeAllCoordinateSystems();
}

void visualisePointClouds(PointCloudPtr cloud1, PointCloudPtr cloud2)
{
  ColorHandlerT green(cloud1, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud1, green, "cloud1");
  ColorHandlerT blue(cloud2, 0.0, 0.0, 255.0);
  vis.addPointCloud(cloud2, blue, "cloud2");
  show();
}

void visualiseTransform(PointCloudPtr cloud1, PointCloudPtr cloud2,
                        const Eigen::Matrix4f &transform)
{
  PointCloudPtr cloud1_aligned(new PointCloud);
  pcl::transformPointCloud(*cloud1, *cloud1_aligned, transform);
  visualisePointClouds(cloud1_aligned, cloud2);
}

void visualisePointCloud(PointCloudPtr cloud)
{
  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");

  show();
}

void visualiseNormals(PointCloudPtr cloud, SurfaceNormalsPtr normals)
{
  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");
  vis.addPointCloudNormals<PointT, NormalT>(cloud, normals, 5, 0.1f, "normals");
  vis.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                                       1.0, 0.0, 0.0, "normals");

  show();
}

void visualiseKeypoints(PointCloudPtr cloud, PointCloudPtr keypoints)
{
  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");

  ColorHandlerT red(keypoints, 255, 0, 0);
  vis.addPointCloud(keypoints, red, "keypoints");
  vis.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints");

  show();
}

void visualiseCorrespondences(PointCloudPtr cloud1, PointCloudPtr keypoints1,
                              PointCloudPtr cloud2, PointCloudPtr keypoints2,
                              CorrespondencesPtr correspondences,
                              bool show_keypoints)
{
  ColorHandlerT green(cloud1, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud1, green, "cloud1");
  ColorHandlerT blue(cloud2, 0.0, 0.0, 255.0);
  vis.addPointCloud(cloud2, blue, "cloud2");

  vis.addCorrespondences<PointT>(keypoints1, keypoints2, *correspondences,
                                 "correspondences");
  vis.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                                  5, "correspondences");

  if (show_keypoints) {
    ColorHandlerT red(keypoints1, 255, 0, 0);
    vis.addPointCloud(keypoints1, red, "keypoints1");
    vis.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints1");

    ColorHandlerT yellow(keypoints2, 255, 255, 0);
    vis.addPointCloud(keypoints2, yellow, "keypoints2");
    vis.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "keypoints2");
  }

  show();
}

#else

void visualisePointClouds(PointCloudPtr cloud1, PointCloudPtr cloud2)
{
}
void visualiseTransform(PointCloudPtr cloud1, PointCloudPtr cloud2,
                        const Eigen::Matrix4f &transform)
{
}
void visualiseCorrespondences(PointCloudPtr cloud1, PointCloudPtr keypoints1,
                              PointCloudPtr cloud2, PointCloudPtr keypoints2,
                              CorrespondencesPtr correspondences,
                              bool show_keypoints)
{
}
void visualisePointCloud(PointCloudPtr cloud)
{
}
void visualiseNormals(PointCloudPtr cloud, SurfaceNormalsPtr normals)
{
}
void visualiseKeypoints(PointCloudPtr cloud, PointCloudPtr keypoints)
{
}

#endif

}  // namespace map_merge_3d
