// nanobind
#include <nanobind/nanobind.h>
#include <nanobind/make_iterator.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// STL
#include <algorithm>
#include <cstdint>
#include <cstring>

namespace nb = nanobind;

namespace {

// Tag types
struct HasXYZ {};
struct NoXYZ {};

// Base binder for point types
template <typename T>
nb::class_<T> bind_point_base(nb::module_ &m, const char *python_name, HasXYZ) {
  auto cls = nb::class_<T>(m, python_name)
          .def(nb::init<>())
          .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"));
  return cls;
}

template <typename T>
nb::class_<T> bind_point_base(nb::module_ &m, const char *python_name, NoXYZ) {
  auto cls = nb::class_<T>(m, python_name)
          .def(nb::init<>());
  return cls;
}

// Field helpers
template <typename T>
nb::class_<T> &add_xyz_fields(nb::class_<T> &cls) {
  return cls.def_rw("x", &T::x).def_rw("y", &T::y).def_rw("z", &T::z);
}

template <typename T>
nb::class_<T> &add_rgba_fields(nb::class_<T> &cls) {
  return cls.def_rw("r", &T::r).def_rw("g", &T::g).def_rw("b", &T::b).def_rw("a", &T::a);
}

template <typename T>
nb::class_<T> &add_rgb_field(nb::class_<T> &cls) {
  return cls.def_rw("rgb", &T::rgb);
}

template <typename T>
nb::class_<T> &add_normal_fields(nb::class_<T> &cls) {
  return cls.def_rw("normal_x", &T::normal_x).def_rw("normal_y", &T::normal_y).def_rw("normal_z", &T::normal_z);
}

// Packed RGB helpers (portable, no UB)
inline std::uint32_t clamp_channel_int(int v) {
  return static_cast<std::uint32_t>(v < 0 ? 0 : (v > 255 ? 255 : v));
}

inline std::uint32_t float_to_u32(float f) {
  std::uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  return u;
}

inline float u32_to_float(std::uint32_t u) {
  float f;
  std::memcpy(&f, &u, sizeof(f));
  return f;
}

// Overloads for types that expose a packed float rgb member
inline nb::class_<pcl::PointXYZRGB> &add_rgb_channel_properties(nb::class_<pcl::PointXYZRGB> &cls) {
  cls.def_prop_rw(
    "r",
    [](const pcl::PointXYZRGB &p) {
      std::uint32_t u = float_to_u32(p.rgb);
      return static_cast<int>((u >> 16) & 0xFFu);
    },
    [](pcl::PointXYZRGB &p, int r) {
      std::uint32_t u = float_to_u32(p.rgb);
      u = (u & 0xFF00FFFFu) | (clamp_channel_int(r) << 16);
      p.rgb = u32_to_float(u);
    },
    "Red channel (0..255) derived from packed rgb");

  cls.def_prop_rw(
    "g",
    [](const pcl::PointXYZRGB &p) {
      std::uint32_t u = float_to_u32(p.rgb);
      return static_cast<int>((u >> 8) & 0xFFu);
    },
    [](pcl::PointXYZRGB &p, int g) {
      std::uint32_t u = float_to_u32(p.rgb);
      u = (u & 0xFFFF00FFu) | (clamp_channel_int(g) << 8);
      p.rgb = u32_to_float(u);
    },
    "Green channel (0..255) derived from packed rgb");

  cls.def_prop_rw(
    "b",
    [](const pcl::PointXYZRGB &p) {
      std::uint32_t u = float_to_u32(p.rgb);
      return static_cast<int>(u & 0xFFu);
    },
    [](pcl::PointXYZRGB &p, int b) {
      std::uint32_t u = float_to_u32(p.rgb);
      u = (u & 0xFFFFFF00u) | clamp_channel_int(b);
      p.rgb = u32_to_float(u);
    },
    "Blue channel (0..255) derived from packed rgb");

  cls.def_prop_rw(
    "rgb_int",
    [](const pcl::PointXYZRGB &p) { return static_cast<std::uint32_t>(float_to_u32(p.rgb) & 0x00FFFFFFu); },
    [](pcl::PointXYZRGB &p, std::uint32_t packed) {
      packed &= 0x00FFFFFFu;
      p.rgb = u32_to_float(packed);
    },
    "Packed RGB as 0xRRGGBB integer");
  return cls;
}

inline nb::class_<pcl::PointXYZRGBNormal> &add_rgb_channel_properties(nb::class_<pcl::PointXYZRGBNormal> &cls) {
  cls.def_prop_rw(
    "r",
    [](const pcl::PointXYZRGBNormal &p) {
      std::uint32_t u = float_to_u32(p.rgb);
      return static_cast<int>((u >> 16) & 0xFFu);
    },
    [](pcl::PointXYZRGBNormal &p, int r) {
      std::uint32_t u = float_to_u32(p.rgb);
      u = (u & 0xFF00FFFFu) | (clamp_channel_int(r) << 16);
      p.rgb = u32_to_float(u);
    },
    "Red channel (0..255) derived from packed rgb");

  cls.def_prop_rw(
    "g",
    [](const pcl::PointXYZRGBNormal &p) {
      std::uint32_t u = float_to_u32(p.rgb);
      return static_cast<int>((u >> 8) & 0xFFu);
    },
    [](pcl::PointXYZRGBNormal &p, int g) {
      std::uint32_t u = float_to_u32(p.rgb);
      u = (u & 0xFFFF00FFu) | (clamp_channel_int(g) << 8);
      p.rgb = u32_to_float(u);
    },
    "Green channel (0..255) derived from packed rgb");

  cls.def_prop_rw(
    "b",
    [](const pcl::PointXYZRGBNormal &p) {
      std::uint32_t u = float_to_u32(p.rgb);
      return static_cast<int>(u & 0xFFu);
    },
    [](pcl::PointXYZRGBNormal &p, int b) {
      std::uint32_t u = float_to_u32(p.rgb);
      u = (u & 0xFFFFFF00u) | clamp_channel_int(b);
      p.rgb = u32_to_float(u);
    },
    "Blue channel (0..255) derived from packed rgb");

  cls.def_prop_rw(
    "rgb_int",
    [](const pcl::PointXYZRGBNormal &p) { return static_cast<std::uint32_t>(float_to_u32(p.rgb) & 0x00FFFFFFu); },
    [](pcl::PointXYZRGBNormal &p, std::uint32_t packed) {
      packed &= 0x00FFFFFFu;
      p.rgb = u32_to_float(packed);
    },
    "Packed RGB as 0xRRGGBB integer");
  return cls;
}

// PointCloud binder with Pythonic conveniences
template <typename PointT>
void bind_pointcloud(nb::module_ &m, const char *python_name) {
  using CloudT = pcl::PointCloud<PointT>;
  nb::class_<CloudT>(m, python_name)
    .def(nb::init<>())
    .def("append", &CloudT::push_back, nb::arg("point"), "Insert a new point at the end of the cloud.")
    .def("at", nb::overload_cast<std::size_t>(&CloudT::at), "Get a point from the cloud by index")
    .def("__getitem__", [](CloudT &v, std::size_t i) -> PointT & { return v[i]; })
    .def("__getitem__", [](CloudT &v, nb::slice s) {
      auto [start, stop, step, length] = s.compute(v.size());
      nb::list out;
      for (std::size_t i = 0, idx = static_cast<std::size_t>(start); i < length; ++i, idx += static_cast<std::size_t>(step))
        out.append(v[idx]);
      return out;
    })
    .def("__len__", [](const CloudT &v) { return v.size(); })
    .def_prop_ro("size", [](const CloudT &v) { return v.size(); }, "Number of points in the cloud")
    .def("resize", nb::overload_cast<std::size_t>(&CloudT::resize))
    .def("reserve", [](CloudT &v, std::size_t n) { v.points.reserve(n); }, nb::arg("n"), "Reserve storage for at least n points")
    .def("extend", [](CloudT &v, nb::iterable it) {
      for (nb::handle h : it)
        v.push_back(nb::cast<PointT>(h));
    }, "Append points from an iterable")
    .def("clear", [](CloudT &v) { v.clear(); })
    .def("__iter__", [](const CloudT &v) {
        return nb::make_iterator(nb::type<CloudT>(), "iterator", v.begin(), v.end());
      }, nb::keep_alive<0, 1>())
    .def_rw("width", &CloudT::width)
    .def_rw("height", &CloudT::height)
    .def_rw("is_dense", &CloudT::is_dense);
}

} // namespace

NB_MODULE(pcl_common_ext, m) {
  // PointXYZ
  {
    auto cls = bind_point_base<pcl::PointXYZ>(m, "PointXYZ", HasXYZ{});
    add_xyz_fields(cls)
      .def("__repr__", [](const pcl::PointXYZ &p) {
        return nb::str("PointXYZ(x={}, y={}, z={})").format(p.x, p.y, p.z);
      });
  }

  // PointXYZRGBA
  {
    auto cls = bind_point_base<pcl::PointXYZRGBA>(m, "PointXYZRGBA", HasXYZ{});
    add_xyz_fields(cls);
    add_rgba_fields(cls)
      .def("__repr__", [](const pcl::PointXYZRGBA &p) {
        return nb::str("PointXYZRGBA(x={}, y={}, z={}, r={}, g={}, b={}, a={})").format(p.x, p.y, p.z, p.r, p.g, p.b, p.a);
      });
  }

  // PointXYZRGB (packed float rgb with channel conveniences)
  {
    auto cls = bind_point_base<pcl::PointXYZRGB>(m, "PointXYZRGB", HasXYZ{});
    add_xyz_fields(cls);
  add_rgb_field(cls);
  add_rgb_channel_properties(cls)
      .def("__repr__", [](const pcl::PointXYZRGB &p) {
        return nb::str("PointXYZRGB(x={}, y={}, z={}, rgb={})").format(p.x, p.y, p.z, p.rgb);
      });
  }

  // PointNormal (named PointXYZNormal for back-compat in this module)
  {
    auto cls = bind_point_base<pcl::PointNormal>(m, "PointXYZNormal", HasXYZ{});
    add_xyz_fields(cls);
    add_normal_fields(cls)
      .def("__repr__", [](const pcl::PointNormal &p) {
        return nb::str("PointXYZNormal(x={}, y={}, z={}, nx={}, ny={}, nz={})").format(p.x, p.y, p.z, p.normal_x, p.normal_y, p.normal_z);
      });
  }

  // PointCloud bindings
  bind_pointcloud<pcl::PointXYZ>(m, "PointcloudXYZ");
  bind_pointcloud<pcl::PointXYZRGBA>(m, "PointcloudXYZRGBA");
  bind_pointcloud<pcl::PointXYZRGB>(m, "PointcloudXYZRGB");
  bind_pointcloud<pcl::PointNormal>(m, "PointcloudXYZNormal");
}
