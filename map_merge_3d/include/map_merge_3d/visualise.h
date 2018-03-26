#ifndef MAP_MERGE_VISUALISE_H_
#define MAP_MERGE_VISUALISE_H_

#include <map_merge_3d/typedefs.h>

void visualisePointClouds(PointCloudPtr cloud1, PointCloudPtr cloud2);
void visualiseCorrespondences(PointCloudPtr cloud1, PointCloudPtr keypoints1,
                              PointCloudPtr cloud2, PointCloudPtr keypoints2,
                              CorrespondencesPtr correspondences,
                              bool show_keypoints = false);
void visualisePointCloud(PointCloudPtr cloud);
void visualiseNormals(PointCloudPtr cloud, SurfaceNormalsPtr normals);
void visualiseKeypoints(PointCloudPtr cloud, PointCloudPtr keypoints);

#endif  // MAP_MERGE_VISUALISE_H_
