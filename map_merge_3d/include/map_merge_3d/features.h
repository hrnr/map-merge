#ifndef MAP_MERGE_FEATURES_H_
#define MAP_MERGE_FEATURES_H_

#include <map_merge_3d/enum.h>
#include <map_merge_3d/typedefs.h>

// define this for dispatch
#define DESCRIPTORS_NAMES_ PFH, PFHRGB, FPFH, RSD, SHOT, SC3D
// define enum class Descriptor + string conversions
ENUM_CLASS(Descriptor, DESCRIPTORS_NAMES_);

PointCloudPtr downSample(const PointCloudConstPtr &input, double resolution);

PointCloudPtr removeOutliers(const PointCloudConstPtr &input, double radius,
                             int min_neighbours);

// defines enum class Keypoint + string conversions
ENUM_CLASS(Keypoint, SIFT, HARRIS);

PointCloudPtr detectKeypoints(const PointCloudConstPtr &points,
                              const SurfaceNormalsPtr &normals, Keypoint type,
                              double threshold, double radius,
                              double resolution);

/**
 * @brief Compute local feature descriptors around each keypoint. If descriptors
 * can not be computer to some of the keypoints, those keypoints will be removed
 * from the keypoints cloud. Therefore the keypoints cloud can be also modified.
 *
 * @param points input pointcloud
 * @param normals input normals for the cloud
 * @param keypoints input detected keypoints, where descriptors will be computed
 * @param feature_radius search radius for descriptors
 * @return cloud of local descriptors
 */
LocalDescriptorsPtr computeLocalDescriptors(const PointCloudConstPtr &points,
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
SurfaceNormalsPtr computeSurfaceNormals(const PointCloudConstPtr &input,
                                        double radius);

#endif  // MAP_MERGE_FEATURES_H_
