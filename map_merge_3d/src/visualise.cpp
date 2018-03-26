#include <map_merge_3d/visualise.h>

#include <pcl/visualization/pcl_visualizer.h>

// use just one instance of vis to overcome bug PCL #172
static pcl::visualization::PCLVisualizer vis("cloud view");

static void show()
{
  vis.spin();
  vis.removeCorrespondences();
  vis.removeAllPointClouds();
  vis.removeAllShapes();
  vis.removeAllCoordinateSystems();
}

void visualisePointClouds(PointCloudPtr cloud1, PointCloudPtr cloud2)
{
  std::cout << "displaying pointclouds of sizes: " << cloud1->size() << ", "
            << cloud2->size() << std::endl;

  ColorHandlerT green(cloud1, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud1, green, "cloud1");
  ColorHandlerT blue(cloud2, 0.0, 0.0, 255.0);
  vis.addPointCloud(cloud2, blue, "cloud2");
  show();
}

void visualisePointCloud(PointCloudPtr cloud)
{
  std::cout << "displaying pointcloud of size: " << cloud->size() << std::endl;

  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");

  show();
}

void visualiseNormals(PointCloudPtr cloud, SurfaceNormalsPtr normals)
{
  std::cout << "displaying pointcloud of size: " << cloud->size() << std::endl;

  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");
  vis.addPointCloudNormals<PointT, NormalT>(cloud, normals, 5, 0.1f, "normals");

  show();
}

void visualiseKeypoints(PointCloudPtr cloud, PointCloudPtr keypoints)
{
  std::cout << "pointcloud size: " << cloud->size() << std::endl
            << "keypoints: " << keypoints->size() << std::endl;

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
  std::cout << "displaying pointclouds of sizes: " << cloud1->size() << ", "
            << cloud2->size() << std::endl;
  std::cout << "keypoints: " << keypoints1->size() << ", " << keypoints2->size()
            << std::endl;
  std::cout << "correspondences: " << correspondences->size() << std::endl;

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
