#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>

namespace nb = nanobind;

template <typename PointT>
void bind_passthrough(nb::module_ &m, const char *py_name) {
  using PT = pcl::PassThrough<PointT>;
  using FI = pcl::FilterIndices<PointT>;
  using FBase = pcl::Filter<PointT>;
  nb::class_<PT>(m, py_name)
    .def(nb::init<>())
    .def("setInputCloud", &PT::setInputCloud, nb::arg("cloud"))
    .def("setFilterLimits", &PT::setFilterLimits, nb::arg("limit_min"), nb::arg("limit_max"))
    .def("setFilterFieldName", &PT::setFilterFieldName, nb::arg("field_name"))
    .def("setNegative", &FI::setNegative, nb::arg("negative"))
    .def("filter", nb::overload_cast<pcl::PointCloud<PointT>&>(&FBase::filter), nb::arg("output"));
}

NB_MODULE(pcl_filters_ext, m) {
#if 0 // abstract class
  nb::class_<pcl::Filter<pcl::PointXYZ>(m, "FilterXYZ")
      .def(nb::init<>())
      .def("filter", &pcl::Filter<pcl::PointXYZ>::filter)
      ;
#endif
#if 0 // abstract class
  nb::class_<pcl::FilterIndices<pcl::PointXYZ>(m, "FilterIndicesXYZ")
      .def(nb::init<>())
      .def("setNegative", &pcl::FilterIndices<pcl::PointXYZ>::setNegative)
      ;
#endif
  bind_passthrough<pcl::PointXYZ>(m, "PassThroughXYZ");
  bind_passthrough<pcl::PointXYZRGBA>(m, "PassThroughXYZRGBA");
  bind_passthrough<pcl::PointXYZRGB>(m, "PassThroughXYZRGB");
  bind_passthrough<pcl::PointNormal>(m, "PassThroughXYZNormal");
  bind_passthrough<pcl::PointXYZRGBNormal>(m, "PassThroughXYZRGBNormal");
}
