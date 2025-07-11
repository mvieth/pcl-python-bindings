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

// Macro to add RGB field (single uint32_t)
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

// Macro to add range field
#define ADD_RANGE_FIELDS(PointType) \
      .def_rw("range", &pcl::PointType::range)

// Macro to add __repr__ for XYZ-only points
#define ADD_XYZ_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={})").format(p.x, p.y, p.z); \
      })

// Macro to add __repr__ for RGBA points
#define ADD_RGBA_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, r={}, g={}, b={}, a={})").format(p.x, p.y, p.z, p.r, p.g, p.b, p.a); \
      })

// Macro to add __repr__ for Normal points
#define ADD_NORMAL_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, nx={}, ny={}, nz={})").format(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z); \
      })

// Macro to add __repr__ for RGBA + Normal points
#define ADD_RGBA_NORMAL_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, r={}, g={}, b={}, a={}, nx={}, ny={}, nz={})").format(p.x, p.y, p.z, p.r, p.g, p.b, p.a, p.normal_x, p.normal_y, p.normal_z); \
      })

// Macro to add __repr__ for Intensity points
#define ADD_INTENSITY_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, intensity={})").format(p.x, p.y, p.z, p.intensity); \
      })

// Macro to add __repr__ for RGB points (no alpha)
#define ADD_RGB_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, rgb={})").format(p.x, p.y, p.z, p.rgb); \
      })

// Macro to add __repr__ for Label points
#define ADD_LABEL_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, label={})").format(p.x, p.y, p.z, p.label); \
      })

// Macro to add __repr__ for points with curvature (Normal)
#define ADD_NORMAL_CURVATURE_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z, p.curvature); \
      })

// Macro to add __repr__ for RGB + Normal + curvature points
#define ADD_RGB_NORMAL_CURVATURE_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, rgb={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.rgb, p.normal_x, p.normal_y, p.normal_z, p.curvature); \
      })

// Macro to add __repr__ for Range points
#define ADD_RANGE_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, range={})").format(p.x, p.y, p.z, p.range); \
      })

// Macro to add __repr__ for RGB + Label points
#define ADD_RGB_LABEL_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, rgb={}, label={})").format(p.x, p.y, p.z, p.rgb, p.label); \
      })

// Macro to add __repr__ for Intensity + Normal points
#define ADD_INTENSITY_NORMAL_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, intensity={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.intensity, p.normal_x, p.normal_y, p.normal_z, p.curvature); \
      })

// Macro to add __repr__ for Label + Normal points
#define ADD_LABEL_NORMAL_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, label={}, nx={}, ny={}, nz={}, curvature={})").format(p.x, p.y, p.z, p.label, p.normal_x, p.normal_y, p.normal_z, p.curvature); \
      })

// Macro to add __repr__ for LAB points
#define ADD_LAB_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, L={}, a={}, b={})").format(p.x, p.y, p.z, p.L, p.a, p.b); \
      })

// Macro to add __repr__ for HSV points
#define ADD_HSV_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, h={}, s={}, v={})").format(p.x, p.y, p.z, p.h, p.s, p.v); \
      })

// Macro to add __repr__ for 2D XY points
#define ADD_XY_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={})").format(p.x, p.y); \
      })

// Macro to add __repr__ for UV points
#define ADD_UV_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(u={}, v={})").format(p.u, p.v); \
      })

// Macro to add __repr__ for Strength points
#define ADD_STRENGTH_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, strength={})").format(p.x, p.y, p.z, p.strength); \
      })

// Macro to add __repr__ for Viewpoint points
#define ADD_VIEWPOINT_REPR(PointType, PythonName) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(x={}, y={}, z={}, vp_x={}, vp_y={}, vp_z={})").format(p.x, p.y, p.z, p.vp_x, p.vp_y, p.vp_z); \
      })

// Composed macros for complete point type bindings
#define BIND_POINT_XYZ(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_XYZ_REPR(PointType, PythonName)

#define BIND_POINT_RGBA(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_RGBA_FIELDS(PointType) \
  ADD_RGBA_REPR(PointType, PythonName)

#define BIND_POINT_RGB(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_RGB_FIELDS(PointType) \
  ADD_RGB_REPR(PointType, PythonName)

#define BIND_POINT_INTENSITY(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_INTENSITY_FIELDS(PointType) \
  ADD_INTENSITY_REPR(PointType, PythonName)

#define BIND_POINT_LABEL(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_LABEL_FIELDS(PointType) \
  ADD_LABEL_REPR(PointType, PythonName)

#define BIND_POINT_NORMAL(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_NORMAL_FIELDS(PointType) \
  ADD_CURVATURE_FIELDS(PointType) \
  ADD_NORMAL_CURVATURE_REPR(PointType, PythonName)

// Special macro for Normal-only points (no XYZ)
#define BIND_NORMAL_ONLY(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      ADD_NORMAL_FIELDS(PointType) \
      ADD_CURVATURE_FIELDS(PointType) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(nx={}, ny={}, nz={}, curvature={})").format(p.normal_x, p.normal_y, p.normal_z, p.curvature); \
      })

#define BIND_POINT_RGB_NORMAL(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_RGB_FIELDS(PointType) \
  ADD_NORMAL_FIELDS(PointType) \
  ADD_CURVATURE_FIELDS(PointType) \
  ADD_RGB_NORMAL_CURVATURE_REPR(PointType, PythonName)

#define BIND_POINT_RGBA_NORMAL(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_RGBA_FIELDS(PointType) \
  ADD_NORMAL_FIELDS(PointType) \
  ADD_RGBA_NORMAL_REPR(PointType, PythonName)

#define BIND_POINT_RANGE(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_RANGE_FIELDS(PointType) \
  ADD_RANGE_REPR(PointType, PythonName)

// Additional bindings for complex point types
#define BIND_POINT_RGB_LABEL(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_RGB_FIELDS(PointType) \
  ADD_LABEL_FIELDS(PointType) \
  ADD_RGB_LABEL_REPR(PointType, PythonName)

#define BIND_POINT_INTENSITY_NORMAL(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_INTENSITY_FIELDS(PointType) \
  ADD_NORMAL_FIELDS(PointType) \
  ADD_CURVATURE_FIELDS(PointType) \
  ADD_INTENSITY_NORMAL_REPR(PointType, PythonName)

#define BIND_POINT_LABEL_NORMAL(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_LABEL_FIELDS(PointType) \
  ADD_NORMAL_FIELDS(PointType) \
  ADD_CURVATURE_FIELDS(PointType) \
  ADD_LABEL_NORMAL_REPR(PointType, PythonName)

#define BIND_POINT_LAB(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_LAB_FIELDS(PointType) \
  ADD_LAB_REPR(PointType, PythonName)

#define BIND_POINT_HSV(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_HSV_FIELDS(PointType) \
  ADD_HSV_REPR(PointType, PythonName)

#define BIND_POINT_XY(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def(nb::init<float, float>(), nb::arg("x"), nb::arg("y")) \
      ADD_XY_FIELDS(PointType) \
      ADD_XY_REPR(PointType, PythonName)

#define BIND_POINT_UV(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def(nb::init<float, float>(), nb::arg("u"), nb::arg("v")) \
      ADD_UV_FIELDS(PointType) \
      ADD_UV_REPR(PointType, PythonName)

#define BIND_POINT_STRENGTH(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_STRENGTH_FIELDS(PointType) \
  ADD_STRENGTH_REPR(PointType, PythonName)

#define BIND_POINT_VIEWPOINT(PointType, PythonName) \
  BIND_POINT_BASE(PointType, PythonName) \
  ADD_XYZ_FIELDS(PointType) \
  ADD_VIEWPOINT_FIELDS(PointType) \
  ADD_VIEWPOINT_REPR(PointType, PythonName)

// Standalone point type macros (no XYZ)
#define BIND_INTENSITY_ONLY(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      ADD_INTENSITY_FIELDS(PointType) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(intensity={})").format(p.intensity); \
      })

#define BIND_LABEL_ONLY(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      ADD_LABEL_FIELDS(PointType) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(label={})").format(p.label); \
      })

#define BIND_RGB_ONLY(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      ADD_RGBA_FIELDS(PointType) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(r={}, g={}, b={}, a={})").format(p.r, p.g, p.b, p.a); \
      })

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

// Macros for feature descriptor types

// Macro for FPFH signature (33 features)
#define BIND_FPFH_SIGNATURE33(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def_prop_rw("histogram", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 33; ++i) result.append(p.histogram[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& hist) { \
                      for (int i = 0; i < 33 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]); \
                    }) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]); \
      })

// Macro for PFH signature (125 features)
#define BIND_PFH_SIGNATURE125(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def_prop_rw("histogram", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 125; ++i) result.append(p.histogram[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& hist) { \
                      for (int i = 0; i < 125 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]); \
                    }) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]); \
      })

// Macro for SHOT signature (352 features)
#define BIND_SHOT_SIGNATURE352(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def_prop_rw("descriptor", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 352; ++i) result.append(p.descriptor[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& desc) { \
                      for (int i = 0; i < 352 && i < desc.size(); ++i) p.descriptor[i] = nb::cast<float>(desc[i]); \
                    }) \
      .def_prop_rw("rf", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 9; ++i) result.append(p.rf[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& rf) { \
                      for (int i = 0; i < 9 && i < rf.size(); ++i) p.rf[i] = nb::cast<float>(rf[i]); \
                    }) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(descriptor[0:5]=[{}, {}, {}, {}, {}]...)").format(p.descriptor[0], p.descriptor[1], p.descriptor[2], p.descriptor[3], p.descriptor[4]); \
      })

// Macro for SHOT signature (1344 features)
#define BIND_SHOT_SIGNATURE1344(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def_prop_rw("descriptor", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 1344; ++i) result.append(p.descriptor[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& desc) { \
                      for (int i = 0; i < 1344 && i < desc.size(); ++i) p.descriptor[i] = nb::cast<float>(desc[i]); \
                    }) \
      .def_prop_rw("rf", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 9; ++i) result.append(p.rf[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& rf) { \
                      for (int i = 0; i < 9 && i < rf.size(); ++i) p.rf[i] = nb::cast<float>(rf[i]); \
                    }) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(descriptor[0:5]=[{}, {}, {}, {}, {}]...)").format(p.descriptor[0], p.descriptor[1], p.descriptor[2], p.descriptor[3], p.descriptor[4]); \
      })

// Macro for VFH signature (308 features)
#define BIND_VFH_SIGNATURE308(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def_prop_rw("histogram", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 308; ++i) result.append(p.histogram[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& hist) { \
                      for (int i = 0; i < 308 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]); \
                    }) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]); \
      })

// Macro for ESF signature (640 features)
#define BIND_ESF_SIGNATURE640(PointType, PythonName) \
  nb::class_<pcl::PointType>(m, PythonName) \
      .def(nb::init()) \
      .def_prop_rw("histogram", \
                    [](const pcl::PointType& p) { \
                      nb::list result; \
                      for (int i = 0; i < 640; ++i) result.append(p.histogram[i]); \
                      return result; \
                    }, \
                    [](pcl::PointType& p, const nb::list& hist) { \
                      for (int i = 0; i < 640 && i < hist.size(); ++i) p.histogram[i] = nb::cast<float>(hist[i]); \
                    }) \
      .def("__repr__", [](const pcl::PointType& p) { \
        return nb::str(PythonName "(histogram[0:5]=[{}, {}, {}, {}, {}]...)").format(p.histogram[0], p.histogram[1], p.histogram[2], p.histogram[3], p.histogram[4]); \
      })

NB_MODULE(pcl_common_ext, m)
{
  // Core point types (known to exist)
  BIND_POINT_XYZ(PointXYZ, "PointXYZ");
  BIND_POINT_RGBA(PointXYZRGBA, "PointXYZRGBA");
  BIND_POINT_RGB(PointXYZRGB, "PointXYZRGB");
  BIND_POINT_INTENSITY(PointXYZI, "PointXYZI");
  BIND_POINT_LABEL(PointXYZL, "PointXYZL");
  
  // Normal types
  BIND_NORMAL_ONLY(Normal, "Normal");
  BIND_POINT_NORMAL(PointNormal, "PointNormal");
  BIND_POINT_RGB_NORMAL(PointXYZRGBNormal, "PointXYZRGBNormal");
  BIND_POINT_INTENSITY_NORMAL(PointXYZINormal, "PointXYZINormal");
  BIND_POINT_LABEL_NORMAL(PointXYZLNormal, "PointXYZLNormal");
  
  // Other types
  BIND_POINT_RANGE(PointWithRange, "PointWithRange");

  // Feature descriptor types
  BIND_FPFH_SIGNATURE33(FPFHSignature33, "FPFHSignature33");
  BIND_PFH_SIGNATURE125(PFHSignature125, "PFHSignature125");
  BIND_SHOT_SIGNATURE352(SHOT352, "SHOT352");
  BIND_SHOT_SIGNATURE1344(SHOT1344, "SHOT1344");
  BIND_VFH_SIGNATURE308(VFHSignature308, "VFHSignature308");
  BIND_ESF_SIGNATURE640(ESFSignature640, "ESFSignature640");

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
