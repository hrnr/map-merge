#ifndef MAP_MERGE_DISPATCH_H_
#define MAP_MERGE_DISPATCH_H_

/// @cond DOXYGEN_SKIP

#include <map_merge_3d/features.h>

#include <tuple>

#include <pcl/features/3dsc.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/rsd.h>
#include <pcl/features/shot.h>

/*
When addding support for a new descriptor: Add descriptor to Descriptor enum in
features.h (though DESCRIPTORS_NAMES_) and declare below required types using
DECLARE_DESCRIPTOR_TYPE. Let the rest be worked out by the macro/template magic.
*/

namespace map_merge_3d
{
// put implementation under anonymous namespace to protect *DescriptorType types
namespace
{
#define DECLARE_DESCRIPTOR_TYPE(type, point_type, estimator, name_)            \
  struct type##DescriptorType {                                                \
    typedef pcl::point_type PointType;                                         \
    typedef pcl::estimator<PointT, NormalT, PointType> Estimator;              \
    constexpr const static auto name = #name_;                                 \
    constexpr const static Descriptor descriptor = Descriptor::type;           \
  };

// all descriptors must also define their signature, estimator and field name in
// PointCloud2 here
DECLARE_DESCRIPTOR_TYPE(PFH, PFHSignature125, PFHEstimation, pfh)
DECLARE_DESCRIPTOR_TYPE(PFHRGB, PFHRGBSignature250, PFHRGBEstimation, pfhrgb)
DECLARE_DESCRIPTOR_TYPE(FPFH, FPFHSignature33, FPFHEstimation, fpfh)
// RIFT uses intensity gradients
// DECLARE_DESCRIPTOR_TYPE(RIFT, Histogram<32>, RIFTEstimation, rift)
DECLARE_DESCRIPTOR_TYPE(RSD, PrincipalRadiiRSD, RSDEstimation, r_min)
// SHOT color descriptor has better performance
// DECLARE_DESCRIPTOR_TYPE(SHOT, SHOT352, SHOTEstimation, shot)
DECLARE_DESCRIPTOR_TYPE(SHOT, SHOT1344, SHOTColorEstimation, shot)
DECLARE_DESCRIPTOR_TYPE(SC3D, ShapeContext1980, ShapeContext3DEstimation,
                        shape_context)

#undef DECLARE_DESCRIPTOR_TYPE
// holds all *DescriptorType
#define APPEND_DESCRIPTOR_TYPE(x) x##DescriptorType
static std::tuple<FOREACH_MACRO(APPEND_DESCRIPTOR_TYPE, DESCRIPTORS_NAMES_)>
    descriptor_types;
#undef APPEND_DESCRIPTOR_TYPE

/* dispatching for all descriptors defined in Descriptor enum */

// default case
template <typename Functor>
static LocalDescriptorsPtr dispatchByDescriptorEnum(Descriptor, Functor)
{
  throw std::runtime_error("unknown descriptor type");
}

template <typename Functor, Descriptor d, Descriptor... D>
static LocalDescriptorsPtr dispatchByDescriptorEnum(Descriptor descriptor,
                                                    Functor f)
{
  if (descriptor == d) {
    return f(std::get<static_cast<size_t>(d)>(descriptor_types));
  }

  return dispatchByDescriptorEnum<Functor, D...>(descriptor, f);
}

/* dispatching based on descriptor field name in PointCloud2 */

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

}  // anonymous namespace

/* this should be used by user */

#define PREPEND_DESCRIPTOR(x) Descriptor::x
template <typename Functor>
static decltype(auto) dispatchForEachDescriptor(const std::string& name,
                                                Functor f)
{
  return dispatchByDescriptorName<Functor, FOREACH_MACRO(PREPEND_DESCRIPTOR,
                                                         DESCRIPTORS_NAMES_)>(
      name, f);
}

template <typename Functor>
static decltype(auto) dispatchForEachDescriptor(Descriptor descriptor,
                                                Functor f)
{
  return dispatchByDescriptorEnum<Functor, FOREACH_MACRO(PREPEND_DESCRIPTOR,
                                                         DESCRIPTORS_NAMES_)>(
      descriptor, f);
}
#undef PREPEND_DESCRIPTOR

}  // namespace map_merge_3d

/// @endcond DOXYGEN_SKIP

#endif  // MAP_MERGE_DISPATCH_H_
