#ifndef MAP_MERGE_VISUALISE_H_
#define MAP_MERGE_VISUALISE_H_

#include <map_merge_3d/typedefs.h>

namespace map_merge_3d
{
void visualisePointClouds(PointCloudPtr cloud1, PointCloudPtr cloud2);
void visualiseTransform(PointCloudPtr cloud1, PointCloudPtr cloud2,
                        const Eigen::Matrix4f &transform);
void visualiseCorrespondences(PointCloudPtr cloud1, PointCloudPtr keypoints1,
                              PointCloudPtr cloud2, PointCloudPtr keypoints2,
                              CorrespondencesPtr correspondences,
                              bool show_keypoints = false);
void visualisePointCloud(PointCloudPtr cloud);
void visualiseNormals(PointCloudPtr cloud, SurfaceNormalsPtr normals);
void visualiseKeypoints(PointCloudPtr cloud, PointCloudPtr keypoints);

}  // namespace map_merge_3d

#endif  // MAP_MERGE_VISUALISE_H_
