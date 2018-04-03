#ifndef MAP_MERGE_DISPATCH_H_
#define MAP_MERGE_DISPATCH_H_

#include <map_merge_3d/features.h>

#include <tuple>

#include <pcl/features/pfh.h>

/*
When addding support for a new descriptor: Add descriptor to DESCRIPTORS_NAMES,
create relevat Descriptor*Type class, and add this class to descriptor types
tuple. Order of the DESCRIPTOR_NAMES must be the same as order of the enum class
and descriptors_types tuple.
*/

// list of all descriptors defined in Descriptor enum
#define DESCRIPTORS_NAMES Descriptor::PFH

struct DescriptorPFHType {
  typedef pcl::PFHSignature125 PointType;
  typedef pcl::PFHEstimation<PointT, NormalT, PointType> Estimator;
  constexpr const static auto name = "pfh";
};

static std::tuple<DescriptorPFHType> descriptor_types;

/* dispatching for all descriptors defined in Descriptor enum */

// default case
template <typename Functor>
LocalDescriptorsPtr dispatch(Descriptor, Functor)
{
  throw std::runtime_error("unknown descriptor type");
}

template <typename Functor, Descriptor d, Descriptor... D>
LocalDescriptorsPtr dispatch(Descriptor descriptor, Functor f)
{
  if (descriptor == d) {
    return f(std::get<static_cast<size_t>(d)>(descriptor_types));
  }

  return dispatch<Functor, D...>(descriptor, f);
}

/* dispatching based on descriptor name */

// default case
template <typename Functor>
auto dispatchByDescriptorName(const std::string&, Functor f)
    -> decltype(f(std::get<0>(descriptor_types)))
{
  throw std::runtime_error("unknown descriptor type");
}

template <typename Functor, Descriptor d, Descriptor... D>
decltype(auto) dispatchByDescriptorName(const std::string& name, Functor f)
{
  auto descriptor_type = std::get<static_cast<size_t>(d)>(descriptor_types);
  if (name == descriptor_type.name) {
    return f(descriptor_type);
  }

  return dispatchByDescriptorName<Functor, D...>(name, f);
}

#endif  // MAP_MERGE_DISPATCH_H_
