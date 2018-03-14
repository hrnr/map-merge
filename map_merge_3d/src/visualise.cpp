#include <map_merge_3d/visualise.h>

#include <pcl/visualization/pcl_visualizer.h>

void visualisePointClouds(PointCloudPtr cloud1, PointCloudPtr cloud2)
{
  std::cout << "displaying pointclouds of sizes: " << cloud1->size() << ", "
            << cloud2->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointclouds match");
  ColorHandlerT green(cloud1, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud1, green, "cloud1");
  ColorHandlerT blue(cloud2, 0.0, 0.0, 255.0);
  vis.addPointCloud(cloud2, blue, "cloud2");
  vis.spin();
  vis.close();
}

void visualisePointCloud(PointCloudPtr cloud)
{
  std::cout << "displaying pointcloud of size: " << cloud->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointcloud");

  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");
  vis.spin();
  vis.close();
  vis.spinOnce();
}

void visualiseNormals(PointCloudPtr cloud, SurfaceNormalsPtr normals)
{
  std::cout << "displaying pointcloud of size: " << cloud->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointcloud and normals");

  ColorHandlerT green(cloud, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud, green, "cloud");
  vis.addPointCloudNormals<PointT, NormalT>(cloud, normals, 5, 0.1f, "normals");

  vis.spin();
  vis.close();
  vis.spinOnce();
}

void visualiseKeypoints(PointCloudPtr cloud, PointCloudPtr keypoints)
{
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

void visualiseCorrespondences(PointCloudPtr cloud1, PointCloudPtr keypoints1,
                              PointCloudPtr cloud2, PointCloudPtr keypoints2,
                              CorrespondencesPtr correspondences)
{
  std::cout << "displaying pointclouds of sizes: " << cloud1->size() << ", "
            << cloud2->size() << std::endl;
  std::cout << "keypoints: " << keypoints1->size() << ", " << keypoints2->size()
            << std::endl;
  std::cout << "correspondences: " << correspondences->size() << std::endl;
  pcl::visualization::PCLVisualizer vis("pointcloud keypoints");

  ColorHandlerT green(cloud1, 0.0, 255.0, 0.0);
  vis.addPointCloud(cloud1, green, "cloud1");
  ColorHandlerT blue(cloud2, 0.0, 0.0, 255.0);
  vis.addPointCloud(cloud2, blue, "cloud2");

  vis.addCorrespondences<PointT>(keypoints1, keypoints2, *correspondences);

  vis.spin();
  vis.close();
  vis.spinOnce();
}
