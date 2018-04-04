#ifndef MAP_MERGE_FEATURES_H_
#define MAP_MERGE_FEATURES_H_

#include <map_merge_3d/typedefs.h>

enum class Descriptor { PFH, FPFH, RSD, SHOT };

Descriptor fromString(const std::string &name);

PointCloudPtr downSample(const PointCloudPtr &input, double resolution);

PointCloudPtr removeOutliers(const PointCloudPtr &input, double radius,
                             int min_neighbours);

/**
 * @brief Use SIFTKeypoint to detect a set of keypoints
 *
 * @param points input cloud
 * @param min_scale the smallest scale in the difference-of-Gaussians (DoG)
 * scale-space
 * @param nr_octaves the number of times the scale doubles in the DoG
 * scale-space
 * @param nr_scales_per_octave the number of scales computed for each doubling
 * @param min_contrast the minimum local contrast that must be present for a
 * keypoint to be detected
 * @return cloud of keypoints
 */
PointCloudPtr detectKeypoints(const PointCloudPtr &points, double min_scale,
                              int nr_octaves, int nr_scales_per_octave,
                              double min_contrast);

/**
 * @brief Compute local feature descriptors around each keypoint
 *
 * @param points input pointcloud
 * @param normals input normals for the cloud
 * @param keypoints input detected keypoints, where descriptors will be computed
 * @param feature_radius search radius for descriptors
 * @return cloud of local descriptors
 */
LocalDescriptorsPtr computeLocalDescriptors(const PointCloudPtr &points,
                                            const SurfaceNormalsPtr &normals,
                                            const PointCloudPtr &keypoints,
                                            Descriptor descriptor,
                                            double feature_radius);

/**
 * @brief Estimate a cloud's surface normals
 *
 * @param input input cloud
 * @param radius local neighbourhood size for estimating normals
 *
 * @return could of computed normals
 */
SurfaceNormalsPtr computeSurfaceNormals(const PointCloudPtr &input,
                                        double radius);

#endif  // MAP_MERGE_FEATURES_H_
