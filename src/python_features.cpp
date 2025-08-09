// Refactored with templates to eliminate repetitive bindings.
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/make_iterator.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>

namespace nb = nanobind;

// Generic NormalEstimation binder (InT -> OutT)
template <typename InT, typename OutT>
void bind_normal_estimation(nb::module_ &m, const char *py_name, const char *doc) {
  using Est = pcl::NormalEstimation<InT, OutT>;
  nb::class_<Est>(m, py_name, doc)
    .def(nb::init<>())
    .def("setInputCloud", &Est::setInputCloud, nb::arg("cloud"))
    .def("setKSearch", &Est::setKSearch, nb::arg("k"), "Set number of nearest neighbors for the estimation.")
    .def("setRadiusSearch", &Est::setRadiusSearch, nb::arg("radius"), "Set spherical neighborhood search radius.")
    .def("compute", &Est::compute, nb::arg("output"), "Estimate normals & curvature into the output cloud.");
}

// Generic FPFH binder; optionally expose setInputNormals depending on whether normals are external.
template <typename PointT, typename NormalT, typename OutT>
void bind_fpfh_estimation(nb::module_ &m, const char *py_name, const char *doc, bool needs_input_normals) {
  using Est = pcl::FPFHEstimation<PointT, NormalT, OutT>;
  auto cls = nb::class_<Est>(m, py_name, doc);
  cls.def(nb::init<>())
     .def("setInputCloud", &Est::setInputCloud, nb::arg("cloud"))
     .def("setKSearch", &Est::setKSearch, nb::arg("k"))
     .def("setRadiusSearch", &Est::setRadiusSearch, nb::arg("radius"))
     .def("compute", &Est::compute, nb::arg("output"));
  if (needs_input_normals) {
    cls.def("setInputNormals", &Est::setInputNormals, nb::arg("normals"));
  }
}

NB_MODULE(pcl_features_ext, m) {
  // Normal estimation variants
  bind_normal_estimation<pcl::PointNormal, pcl::PointNormal>(
    m, "NormalEstimationXYZNormal",
    "NormalEstimation (PointXYZNormal -> PointXYZNormal) computing/overwriting normals.");
  bind_normal_estimation<pcl::PointXYZ, pcl::PointNormal>(
    m, "NormalEstimationXYZ",
    "NormalEstimation (PointXYZ -> PointXYZNormal) producing a new cloud with normals.");
  bind_normal_estimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal>(
    m, "NormalEstimationXYZRGB",
    "NormalEstimation (PointXYZRGB -> PointXYZRGBNormal) producing a new cloud with normals.");

  // Descriptor point type & cloud (FPFHSignature33)
  using FPFHPoint = pcl::FPFHSignature33;
  using FPFHCloud = pcl::PointCloud<FPFHPoint>;
  nb::class_<FPFHPoint>(m, "FPFHSignature33")
    .def(nb::init<>())
    .def_prop_rw(
      "hist",
      [](FPFHPoint &p){ nb::list out; for(int i=0;i<33;++i) out.append(p.histogram[i]); return out; },
      [](FPFHPoint &p, nb::handle seq){
        nb::list l(seq);
        if (l.size() != 33) throw std::runtime_error("Expected 33 values");
        for (int i=0;i<33;++i) p.histogram[i] = nb::cast<float>(l[i]);
      },
      "33-bin FPFH histogram as a Python list");
  nb::class_<FPFHCloud>(m, "PointcloudFPFHSignature33")
    .def(nb::init<>())
    .def("append", [](FPFHCloud &c, const FPFHPoint &pt){ c.push_back(pt); })
    .def("__len__", [](const FPFHCloud &c){ return c.size(); })
  .def("__getitem__", [](FPFHCloud &c, std::size_t i) -> FPFHPoint& { return c[i]; })
    .def("__iter__", [](FPFHCloud &c){ return nb::make_iterator(nb::type<FPFHCloud>(), "fpfh_iter", c.begin(), c.end()); }, nb::keep_alive<0,1>());

  // FPFH variants (keep set consistent with Python facade)
  bind_fpfh_estimation<pcl::PointXYZ, pcl::Normal, FPFHPoint>(
    m, "FPFHEstimationXYZ", "FPFH estimation for PointXYZ using an external normals cloud.", true);
  bind_fpfh_estimation<pcl::PointXYZRGB, pcl::Normal, FPFHPoint>(
    m, "FPFHEstimationXYZRGB", "FPFH estimation for PointXYZRGB using an external normals cloud.", true);
  bind_fpfh_estimation<pcl::PointNormal, pcl::PointNormal, FPFHPoint>(
    m, "FPFHEstimationXYZNormal", "FPFH estimation where the input point type already contains normals.", false);
}
