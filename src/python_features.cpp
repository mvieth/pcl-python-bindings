#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>

namespace nb = nanobind;

NB_MODULE(pcl_features_ext, m)
{
  nb::class_<pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>>(m, "NormalEstimationXYZNormal") // TODO to discuss: naming: how to deal with two different template parameters?
      .def(nb::init<>())
      .def("setInputCloud", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::setInputCloud) // TODO maybe inherit this from PCLBase?
      .def("setKSearch", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::setKSearch)
      .def("setRadiusSearch", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::setRadiusSearch)
      .def("compute", &pcl::NormalEstimation<pcl::PointNormal, pcl::PointNormal>::compute)
      ;
}
