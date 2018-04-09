#include <map_merge_3d/map_merging.h>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char **argv)
{
  std::vector<int> pcd_file_indices =
      pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
  std::string output_name = "output.pcd";

  if (pcd_file_indices.size() < 2) {
    pcl::console::print_error("Need at least 2 input files!\n");
    return -1;
  }

  // load input pointclouds
  std::vector<PointCloudPtr> clouds;
  for (int idx : pcd_file_indices) {
    PointCloudPtr cloud(new PointCloud);
    auto file_name = argv[idx];
    if (pcl::io::loadPCDFile<PointT>(file_name, *cloud) < 0) {
      pcl::console::print_error("Error loading pointcloud file %s. Aborting.\n",
                                file_name);
      return -1;
    }
    clouds.push_back(cloud);
  }

  return 0;
}
