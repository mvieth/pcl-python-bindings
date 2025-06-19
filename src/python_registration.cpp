#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

namespace nb = nanobind;

NB_MODULE(pcl_registration_ext, m)
{
  nb::class_<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>>(m, "IterativeClosestPointXYZ") // TODO to discuss: Scalar=float or =double?
      .def(nb::init<>())
      .def("getFinalTransformation", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::getFinalTransformation, "Get the final transformation matrix estimated by the registration method.")
      .def("hasConverged", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::hasConverged, "Return the state of convergence after the last align run.")
      .def("setInputSource", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::setInputSource, nb::arg("cloud"), "Provide a pointer to the input source (e.g., the point cloud that we want to align to the target)")
      .def("setInputTarget", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::setInputTarget, nb::arg("cloud"), "Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)")
      .def("align", nb::overload_cast<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::PointCloudSource&>(&pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::align), nb::arg("output"), "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.")
      //.def("align", nb::overload_cast<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::PointCloudSource&, const pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4&>(&pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::align), nb::arg("output"), nb::arg("guess")=pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4::Identity(), "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.")
      ;
}
