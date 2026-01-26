#include <nanobind/nanobind.h>
#include <nanobind/make_iterator.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace nb = nanobind;

// Base macro for starting any point type binding
#define BIND_POINT_BASE(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))

// Base macro for non-XYZ point types (like Normal)
#define BIND_POINT_BASE_NO_XYZ(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init())

// Macro to add XYZ coordinates (reusable for all point types)
#define ADD_XYZ_FIELDS(PointType) \
      .def_rw("x", &pcl::PointType::x) \
      .def_rw("y", &pcl::PointType::y) \
      .def_rw("z", &pcl::PointType::z)

// Macro to add RGBA color fields
#define ADD_RGBA_FIELDS(PointType) \
      .def_rw("r", &pcl::PointType::r) \
      .def_rw("g", &pcl::PointType::g) \
      .def_rw("b", &pcl::PointType::b) \
      .def_rw("a", &pcl::PointType::a)

// Macro to add RGB field (packed as float)
#define ADD_RGB_FIELDS(PointType) \
      .def_rw("rgb", &pcl::PointType::rgb)

// Macro to add normal vector fields
#define ADD_NORMAL_FIELDS(PointType) \
      .def_rw("normal_x", &pcl::PointType::normal_x) \
      .def_rw("normal_y", &pcl::PointType::normal_y) \
      .def_rw("normal_z", &pcl::PointType::normal_z)

// Macro to add intensity field
#define ADD_INTENSITY_FIELDS(PointType) \
      .def_rw("intensity", &pcl::PointType::intensity)

// Macro to add label field
#define ADD_LABEL_FIELDS(PointType) \
      .def_rw("label", &pcl::PointType::label)

// Macro to add curvature field
#define ADD_CURVATURE_FIELDS(PointType) \
      .def_rw("curvature", &pcl::PointType::curvature)

// Macro for binding point cloud types
#define BIND_POINTCLOUD(PointType, PythonName) \
  nb::class_<pcl::PointCloud<pcl::PointType>>(m, PythonName) \
      .def(nb::init<>()) \
      .def("append", &pcl::PointCloud<pcl::PointType>::push_back, nb::arg("point"), "Insert a new point in the cloud, at the end of the container.") \
      .def("at", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointType>::at), "Get a point from the cloud") \
      .def("__getitem__", \
           [](pcl::PointCloud<pcl::PointType>& cloud, std::size_t i) -> pcl::PointType& { \
             return cloud[i]; \
           }) \
      .def("__len__", [](const pcl::PointCloud<pcl::PointType>& v) { return v.size(); }) \
      .def("resize", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointType>::resize)) \
      .def("clear", [](pcl::PointCloud<pcl::PointType>& v) { v.clear(); }) \
      .def("__iter__", \
           [](const pcl::PointCloud<pcl::PointType>& v) { \
             return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointType>>(), "iterator", \
                                      v.begin(), v.end()); \
           }, nb::keep_alive<0, 1>())

NB_MODULE(pcl_common_ext, m)
{
  // Core point types (known to exist)
  BIND_POINT_BASE(PointXYZ, "PointXYZ")
  ADD_XYZ_FIELDS(PointXYZ)
      .def("__repr__", [](const pcl::PointXYZ& p) {
        return nb::str("PointXYZ" "(x={}, y={}, z={})").format(p.x, p.y, p.z);
      });

  BIND_POINT_BASE(PointXYZRGBA, "PointXYZRGBA")
  ADD_XYZ_FIELDS(PointXYZRGBA)
  ADD_RGBA_FIELDS(PointXYZRGBA)
      .def("__repr__", [](const pcl::PointXYZRGBA& p) {
        return nb::str("PointXYZRGBA" "(x={}, y={}, z={}, r={}, g={}, b={}, a={})").format(p.x, p.y, p.z, p.r, p.g, p.b, p.a);
      });

  BIND_POINT_BASE(PointXYZRGB, "PointXYZRGB")
  ADD_XYZ_FIELDS(PointXYZRGB)
  ADD_RGB_FIELDS(PointXYZRGB)
      .def("__repr__", [](const pcl::PointXYZRGB& p) {
        return nb::str("PointXYZRGB" "(x={}, y={}, z={}, rgb={})").format(p.x, p.y, p.z, p.rgb);
      });

  BIND_POINT_BASE(PointXYZI, "PointXYZI")
  ADD_XYZ_FIELDS(PointXYZI)
  ADD_INTENSITY_FIELDS(PointXYZI)
      .def("__repr__", [](const pcl::PointXYZI& p) {
        return nb::str("PointXYZI" "(x={}, y={}, z={}, intensity={})").format(p.x, p.y, p.z, p.intensity);
      });

  BIND_POINT_BASE(PointXYZL, "PointXYZL")
  ADD_XYZ_FIELDS(PointXYZL)
  ADD_LABEL_FIELDS(PointXYZL)
      .def("__repr__", [](const pcl::PointXYZL& p) {
        return nb::str("PointXYZL" "(x={}, y={}, z={}, label={})").format(p.x, p.y, p.z, p.label);
      });
  
  // Normal types
  BIND_POINT_BASE_NO_XYZ(Normal, "Normal")
      ADD_NORMAL_FIELDS(Normal)
      ADD_CURVATURE_FIELDS(Normal)
      .def("__repr__", [](const pcl::Normal& p) {
        return nb::str("Normal" "(nx={}, ny={}, nz={}, curvature={})").format(p.normal_x, p.normal_y, p.normal_z, p.curvature);
      });

  BIND_POINT_BASE(PointNormal, "PointNormal")
  ADD_XYZ_FIELDS(PointNormal)
  ADD_NORMAL_FIELDS(PointNormal)
  ADD_CURVATURE_FIELDS(PointNormal)
      .def("__repr__", [](const pcl::PointNormal& p) {
        return nb::str("PointNormal" "(x={}, y={}, z={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature);
      });

  BIND_POINT_BASE(PointXYZRGBNormal, "PointXYZRGBNormal")
  ADD_XYZ_FIELDS(PointXYZRGBNormal)
  ADD_RGB_FIELDS(PointXYZRGBNormal)
  ADD_NORMAL_FIELDS(PointXYZRGBNormal)
  ADD_CURVATURE_FIELDS(PointXYZRGBNormal)
      .def("__repr__", [](const pcl::PointXYZRGBNormal& p) {
        return nb::str("PointXYZRGBNormal" "(x={}, y={}, z={}, rgb={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.rgb, p.normal_x, p.normal_y, p.normal_z, p.curvature);
      });

  BIND_POINT_BASE(PointXYZINormal, "PointXYZINormal")
  ADD_XYZ_FIELDS(PointXYZINormal)
  ADD_INTENSITY_FIELDS(PointXYZINormal)
  ADD_NORMAL_FIELDS(PointXYZINormal)
  ADD_CURVATURE_FIELDS(PointXYZINormal)
      .def("__repr__", [](const pcl::PointXYZINormal& p) {
        return nb::str("PointXYZINormal" "(x={}, y={}, z={}, intensity={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.intensity, p.normal_x, p.normal_y, p.normal_z, p.curvature);
      });

  BIND_POINT_BASE(PointXYZLNormal, "PointXYZLNormal")
  ADD_XYZ_FIELDS(PointXYZLNormal)
  ADD_LABEL_FIELDS(PointXYZLNormal)
  ADD_NORMAL_FIELDS(PointXYZLNormal)
  ADD_CURVATURE_FIELDS(PointXYZLNormal)
      .def("__repr__", [](const pcl::PointXYZLNormal& p) {
        return nb::str("PointXYZLNormal" "(x={}, y={}, z={}, label={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.label, p.normal_x, p.normal_y, p.normal_z, p.curvature);
      });
  
  // Other types
  BIND_POINT_BASE(PointWithRange, "PointWithRange")
  ADD_XYZ_FIELDS(PointWithRange)
      .def_rw("range", &pcl::PointWithRange::range)
      .def("__repr__", [](const pcl::PointWithRange& p) {
        return nb::str("PointWithRange" "(x={}, y={}, z={}, range={})").format(p.x, p.y, p.z, p.range);
      });

  // Feature descriptor types
  BIND_POINT_BASE_NO_XYZ(FPFHSignature33, "FPFHSignature33")
      .def_prop_rw("histogram",
                    [](const pcl::FPFHSignature33& p) {
                      nb::list result;
                      for (int i = 0; i < 33; ++i) result.append(p.histogram[i]);
                      return result;
                    },
                    [](pcl::FPFHSignature33& p, const nb::list& hist) {
                      for (int i = 0; i < 33 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]);
                    })
      .def("__repr__", [](const pcl::FPFHSignature33& p) {
        return nb::str("FPFHSignature33" "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]);
      });

  BIND_POINT_BASE_NO_XYZ(PFHSignature125, "PFHSignature125")
      .def_prop_rw("histogram",
                    [](const pcl::PFHSignature125& p) {
                      nb::list result;
                      for (int i = 0; i < 125; ++i) result.append(p.histogram[i]);
                      return result;
                    },
                    [](pcl::PFHSignature125& p, const nb::list& hist) {
                      for (int i = 0; i < 125 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]);
                    })
      .def("__repr__", [](const pcl::PFHSignature125& p) {
        return nb::str("PFHSignature125" "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]);
      });

  BIND_POINT_BASE_NO_XYZ(SHOT352, "SHOT352")
      .def_prop_rw("descriptor",
                    [](const pcl::SHOT352& p) {
                      nb::list result;
                      for (int i = 0; i < 352; ++i) result.append(p.descriptor[i]);
                      return result;
                    },
                    [](pcl::SHOT352& p, const nb::list& desc) {
                      for (int i = 0; i < 352 && i < desc.size(); ++i) p.descriptor[i] = nb::cast<float>(desc[i]);
                    })
      .def_prop_rw("rf",
                    [](const pcl::SHOT352& p) {
                      nb::list result;
                      for (int i = 0; i < 9; ++i) result.append(p.rf[i]);
                      return result;
                    },
                    [](pcl::SHOT352& p, const nb::list& rf) {
                      for (int i = 0; i < 9 && i < rf.size(); ++i) p.rf[i] = nb::cast<float>(rf[i]);
                    })
      .def("__repr__", [](const pcl::SHOT352& p) {
        return nb::str("SHOT352" "(descriptor[0:5]=[{}, {}, {}, {}, {}]...)").format(p.descriptor[0], p.descriptor[1], p.descriptor[2], p.descriptor[3], p.descriptor[4]);
      });

  BIND_POINT_BASE_NO_XYZ(SHOT1344, "SHOT1344")
      .def_prop_rw("descriptor",
                    [](const pcl::SHOT1344& p) {
                      nb::list result;
                      for (int i = 0; i < 1344; ++i) result.append(p.descriptor[i]);
                      return result;
                    },
                    [](pcl::SHOT1344& p, const nb::list& desc) {
                      for (int i = 0; i < 1344 && i < desc.size(); ++i) p.descriptor[i] = nb::cast<float>(desc[i]);
                    })
      .def_prop_rw("rf",
                    [](pcl::SHOT1344& p) {
                      nb::list result;
                      for (int i = 0; i < 9; ++i) result.append(p.rf[i]);
                      return result;
                    },
                    [](pcl::SHOT1344& p, const nb::list& rf) {
                      for (int i = 0; i < 9 && i < rf.size(); ++i) p.rf[i] = nb::cast<float>(rf[i]);
                    })
      .def("__repr__", [](const pcl::SHOT1344& p) {
        return nb::str("SHOT1344" "(descriptor[0:5]=[{}, {}, {}, {}, {}]...)").format(p.descriptor[0], p.descriptor[1], p.descriptor[2], p.descriptor[3], p.descriptor[4]);
      });

  BIND_POINT_BASE_NO_XYZ(VFHSignature308, "VFHSignature308")
      .def_prop_rw("histogram",
                    [](const pcl::VFHSignature308& p) {
                      nb::list result;
                      for (int i = 0; i < 308; ++i) result.append(p.histogram[i]);
                      return result;
                    },
                    [](pcl::VFHSignature308& p, const nb::list& hist) {
                      for (int i = 0; i < 308 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]);
                    })
      .def("__repr__", [](const pcl::VFHSignature308& p) {
        return nb::str("VFHSignature308" "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]);
      });

  BIND_POINT_BASE_NO_XYZ(ESFSignature640, "ESFSignature640")
      .def_prop_rw("histogram",
                    [](const pcl::ESFSignature640& p) {
                      nb::list result;
                      for (int i = 0; i < 640; ++i) result.append(p.histogram[i]);
                      return result;
                    },
                    [](pcl::ESFSignature640& p, const nb::list& hist) {
                      for (int i = 0; i < 640 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]);
                    })
      .def("__repr__", [](const pcl::ESFSignature640& p) {
        return nb::str("ESFSignature640" "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]);
      });

  // Point cloud types for existing types
  BIND_POINTCLOUD(PointXYZ, "PointcloudXYZ");
  BIND_POINTCLOUD(PointXYZRGBA, "PointcloudXYZRGBA");
  BIND_POINTCLOUD(PointXYZRGB, "PointcloudXYZRGB");
  BIND_POINTCLOUD(PointXYZI, "PointcloudXYZI");
  BIND_POINTCLOUD(PointXYZL, "PointcloudXYZL");
  BIND_POINTCLOUD(Normal, "PointcloudNormal");
  BIND_POINTCLOUD(PointNormal, "PointcloudPointNormal");
  BIND_POINTCLOUD(PointXYZRGBNormal, "PointcloudXYZRGBNormal");
  BIND_POINTCLOUD(PointXYZINormal, "PointcloudXYZINormal");
  BIND_POINTCLOUD(PointXYZLNormal, "PointcloudXYZLNormal");
  BIND_POINTCLOUD(PointWithRange, "PointcloudWithRange");

  // Point cloud types for feature descriptors
  BIND_POINTCLOUD(FPFHSignature33, "PointcloudFPFHSignature33");
  BIND_POINTCLOUD(PFHSignature125, "PointcloudPFHSignature125");
  BIND_POINTCLOUD(SHOT352, "PointcloudSHOT352");
  BIND_POINTCLOUD(SHOT1344, "PointcloudSHOT1344");
  BIND_POINTCLOUD(VFHSignature308, "PointcloudVFHSignature308");
  BIND_POINTCLOUD(ESFSignature640, "PointcloudESFSignature640");
}
