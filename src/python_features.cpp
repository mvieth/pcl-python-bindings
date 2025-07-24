#include <variant>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include "bind_utils.hpp"

namespace nb = nanobind;
using namespace nb::literals;

template<typename T>
std::shared_ptr<T> create_shared_ptr(T& value){
  std::shared_ptr<T> pointer(&value);
  return pointer;
};

class NormalEstimation{
  public:
    int k_search;
    double radius_search;

    NormalEstimation(int k_search, double radius_search): k_search(k_search), radius_search(radius_search) {}
};

template<typename T>
pcl::NormalEstimation<T, T> create_estimator(T&){
  return pcl::NormalEstimation<T, T>();
};

template<typename T> requires (!HasPosition<T>)
void compute_inplace(T& cloud, NormalEstimation&) {std::cout << "Does not have position.\n"; throw 101;};
template<typename T> requires (!HasNormal<T>)
void compute_inplace(T& cloud, NormalEstimation&) {std::cout << "Does not have normal.\n"; throw 101;};
template<typename T> requires (HasPosition<T> && HasNormal<T>)
void compute_inplace(T& cloud, const NormalEstimation& parameters){
  // Pass point cloud to infer reference type
  auto estimator = create_estimator(cloud.front());
  estimator.setKSearch(parameters.k_search);
  estimator.setRadiusSearch(parameters.radius_search);
  auto cloud_ptr = create_shared_ptr(cloud);
  estimator.setInputCloud(cloud_ptr);
  estimator.compute(cloud);
};

NB_MODULE(pcl_features_ext, m)
{
  nb::class_<NormalEstimation>(m, "NormalEstimation", 
    "NormalEstimation estimates local surface properties (surface normals and curvatures) at each 3D point.")
  .def(nb::init<int, double>(), "k_search"_a= 0, "radius_search"_a = 0.0)
  .def_rw("k_search", &NormalEstimation::k_search, 
    "Set the number of k nearest neighbors to use for the feature estimation.")
  .def_rw("radius_search", &NormalEstimation::radius_search, 
    "Set the sphere radius that is to be used for determining the nearest neighbors used for the feature estimation.")
  .def("compute", [](NormalEstimation& estimation, PointCloud& cloud){
    std::visit([&estimation](auto& arg){
      compute_inplace(arg, estimation);
    }, cloud.data);
  },
    "Estimate normals and curvatures and store them in the given output cloud.")
  ;

  // TODO what about NormalEstimationOMP? Maybe rather expose that one in Python?
  nb::class_<pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>>(m, "NormalEstimationXYZNormal", "NormalEstimation estimates local surface properties (surface normals and curvatures) at each 3D point.") // TODO to discuss: naming: how to deal with two different template parameters?
      .def(nb::init<>())
      .def("setInputCloud", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::setInputCloud, nb::arg("cloud")) // TODO maybe inherit this from PCLBase?
      .def("setKSearch", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::setKSearch, nb::arg("k"), "Set the number of k nearest neighbors to use for the feature estimation.")
      .def("setRadiusSearch", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::setRadiusSearch, nb::arg("radius"), "Set the sphere radius that is to be used for determining the nearest neighbors used for the feature estimation.")
      .def("compute", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::compute, nb::arg("output"), "Estimate normals and curvatures and store them in the given output cloud.")
      ;
}
