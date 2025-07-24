#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

#include "bind_utils.hpp"

namespace nb = nanobind;

struct NormalEstimation{
  int k_search = 0;
  double radius_search = 0.0;
};

NB_MODULE(pcl_features_ext, m)
{
  nb::class_<NormalEstimation>(m, "NormalEstimation", "NormalEstimation estimates local surface properties (surface normals and curvatures) at each 3D point.")
  .def(nb::init<>(&add, "k_search"_a = 0, "radius_search"_a = 0.0))
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
