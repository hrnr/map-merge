#ifndef MAP_MERGE_DISPATCH_H_
#define MAP_MERGE_DISPATCH_H_

#include <map_merge_3d/features.h>

#include <tuple>

#include <pcl/features/3dsc.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/rsd.h>
#include <pcl/features/shot.h>

/*
When addding support for a new descriptor: Add descriptor to DESCRIPTORS_NAMES,
create relevat Descriptor*Type class, and add this class to descriptor types
tuple. Order of the DESCRIPTOR_NAMES must be the same as order of the enum class
and descriptors_types tuple.
*/

// list of all descriptors defined in Descriptor enum
#define DESCRIPTORS_NAMES                                                      \
  Descriptor::PFH, Descriptor::PFHRGB, Descriptor::FPFH, Descriptor::RSD,      \
      Descriptor::SHOT, Descriptor::SC3D

#define DECLARE_DESCRIPTOR_TYPE(type, point_type, estimator, name_)            \
  struct type##DescriptorType {                                                \
    typedef pcl::point_type PointType;                                         \
    typedef pcl::estimator<PointT, NormalT, PointType> Estimator;              \
    constexpr const static auto name = #name_;                                 \
    constexpr const static Descriptor descriptor = Descriptor::type;           \
  };

DECLARE_DESCRIPTOR_TYPE(PFH, PFHSignature125, PFHEstimation, pfh)
DECLARE_DESCRIPTOR_TYPE(PFHRGB, PFHRGBSignature250, PFHRGBEstimation, pfhrgb)
DECLARE_DESCRIPTOR_TYPE(FPFH, FPFHSignature33, FPFHEstimation, fpfh)
// RIFT uses intensity gradients
// DECLARE_DESCRIPTOR_TYPE(RIFT, Histogram<32>, RIFTEstimation, rift)
DECLARE_DESCRIPTOR_TYPE(RSD, PrincipalRadiiRSD, RSDEstimation, r_min)
// DECLARE_DESCRIPTOR_TYPE(SHOT, SHOT352, SHOTEstimation, shot)
// SHOT color descriptor is indistinguishable from normal shot
DECLARE_DESCRIPTOR_TYPE(SHOT, SHOT1344, SHOTColorEstimation, shot)
DECLARE_DESCRIPTOR_TYPE(SC3D, ShapeContext1980, ShapeContext3DEstimation,
                        shape_context)

static std::tuple<PFHDescriptorType, PFHRGBDescriptorType, FPFHDescriptorType,
                  RSDDescriptorType, SHOTDescriptorType, SC3DDescriptorType>
    descriptor_types;

/* dispatching for all descriptors defined in Descriptor enum */

// default case
template <typename Functor>
static LocalDescriptorsPtr dispatch(Descriptor, Functor)
{
  throw std::runtime_error("unknown descriptor type");
}

template <typename Functor, Descriptor d, Descriptor... D>
static LocalDescriptorsPtr dispatch(Descriptor descriptor, Functor f)
{
  if (descriptor == d) {
    return f(std::get<static_cast<size_t>(d)>(descriptor_types));
  }

  return dispatch<Functor, D...>(descriptor, f);
}

/* dispatching based on descriptor name */

// default case
template <typename Functor>
static auto dispatchByDescriptorName(const std::string&, Functor f)
    -> decltype(f(std::get<0>(descriptor_types)))
{
  throw std::runtime_error("unknown descriptor type");
}

template <typename Functor, Descriptor d, Descriptor... D>
static decltype(auto) dispatchByDescriptorName(const std::string& name,
                                               Functor f)
{
  auto descriptor_type = std::get<static_cast<size_t>(d)>(descriptor_types);
  if (name == descriptor_type.name) {
    return f(descriptor_type);
  }

  return dispatchByDescriptorName<Functor, D...>(name, f);
}

#endif  // MAP_MERGE_DISPATCH_H_
