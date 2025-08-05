#include <string>
#include <variant>

#include <nanobind/make_iterator.h>
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "bind_utils.hpp"

namespace nb = nanobind;

template<typename T> requires (!HasPosition<T>)
void set_positions(T &cloud, InputArray2d array)  {std::cout << "Key does not exist.\n"; throw 101;};
template<typename T> requires HasPosition<T>
void set_positions(T &cloud, InputArray2d array)
{
  cloud->resize(array.shape(0));

  for (int ii = 0; ii < array.shape(0); ii++){
    (*cloud)[ii].x = array(ii, 0);
    (*cloud)[ii].y = array(ii, 1);
    (*cloud)[ii].z = array(ii, 2);
  }
};

template<typename T> requires (!HasPosition<T>)
NumpyArray2d get_positions(T &cloud) {std::cout << "Key does not exist.\n"; throw 101;};
template<typename T> requires HasPosition<T>
NumpyArray2d get_positions(T &cloud){
  size_t cols = 3;
  auto rows = cloud->size(); 

  float *data = new float[rows * cols];
  for (size_t i = 0; i < rows; ++i){
    auto value = (*cloud)[i];
    data[i * cols]  = value.x;
    data[i * cols + 1]  = value.y;
    data[i * cols + 2]  = value.z;
  }

  auto owner = delete_owner(data);

  return NumpyArray2d(data,{ rows, cols }, owner);
};

template<typename T> requires (!HasNormal<T>)
void set_normals(T &cloud, const InputArray2d& array)  {std::cout << "Key does not exist.\n"; throw 101;};
template<typename T> requires HasNormal<T>
void set_normals(T &cloud, const InputArray2d& array)
{
  cloud->resize(array.shape(0));

  for (int ii = 0; ii < array.shape(0); ii++){
    (*cloud)[ii].normal_x = array(ii, 0);
    (*cloud)[ii].normal_y = array(ii, 1);
    (*cloud)[ii].normal_z = array(ii, 2);
  }
};

template<typename T> requires (!HasNormal<T>)
NumpyArray2d get_normals(T &cloud) {std::cout << "Key does not exist.\n"; throw 101;};
template<typename T> requires HasNormal<T>
NumpyArray2d get_normals(T &cloud){
  size_t cols = 3;
  auto rows = cloud->size(); 

  float *data = new float[rows * cols];
  for (size_t i = 0; i < rows; ++i){
    auto value = (*cloud)[i];
    data[i * cols]  = value.normal_x;
    data[i * cols + 1]  = value.normal_y;
    data[i * cols + 2]  = value.normal_z;
  }

  auto owner = delete_owner(data);

  return NumpyArray2d(data,{ rows, cols }, owner);
};


NB_MODULE(pcl_common_ext, m)
{
  nb::enum_<PointType>(m, "PointType")
  .value("PointXYZ", PointXYZ)
  .value("PointXYZI", PointXYZI)
  .value("PointXYZL", PointXYZL)
  .value("PointXYZRGBA", PointXYZRGBA)
  .value("PointXYZRGB", PointXYZRGB)
  .value("PointXYZRGBL", PointXYZRGBL)
  .value("PointXYZLAB", PointXYZLAB)
  .value("PointXYZHSV", PointXYZHSV)
  .value("Normal", Normal)
  .value("PointNormal", PointNormal)
  .value("PointXYZRGBNormal", PointXYZRGBNormal)
  .value("PointXYZINormal", PointXYZINormal)
  .value("PointXYZLNormal", PointXYZLNormal)
  .export_values();

  nb::class_<PointCloud>(m, "PointCloud")
    .def(nb::init<PointType>())
    .def(nb::init<CloudVariant>())
    .def("__repr__", [](const PointCloud& cloud) {
      return nb::str("PointCloud<{}> of length {} with keys:\n {}").format(
        cloud.type(), cloud.size(), keys::get_keys(cloud));
    })
    .def("keys", &keys::get_keys)
    .def_prop_ro("type", [](PointCloud &cloud) { return cloud.type();})
    .def("__len__", &PointCloud::size)
    .def("__setitem__", [](PointCloud& cloud, std::string key, InputArray2d array){
      return std::visit([key, array](auto&& arg){ 
        if (key == keys::position){
          return set_positions(arg, array);
        } else if (key == keys::normal){
          return set_normals(arg, array);
        } else {
          std::cout << "Unknown key " << key << "\n";
          throw 101;
        }
      }, 
      cloud.data);
    }
  )
  .def("__getitem__", [](PointCloud& cloud, std::string key){
      return std::visit([key](auto&& arg){ 
        if (key == keys::position){
          return get_positions(arg);
        } else if (key == keys::normal){
          return get_normals(arg);
        } else {
          std::cout << "Unknown key " << key << "\n";
          throw 101;
        }
      }, 
      cloud.data);
    }
  )
  ;

  nb::class_<pcl::PointXYZ>(m, "PointXYZ")
      .def(nb::init())
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
      .def_rw("x", &pcl::PointXYZ::x)
      .def_rw("y", &pcl::PointXYZ::y)
      .def_rw("z", &pcl::PointXYZ::z)
      .def("__repr__", [](const pcl::PointXYZ& p) {
        return nb::str("PointXYZ(x={}, y={}, z={})").format(p.x, p.y, p.z);
      });

  nb::class_<pcl::PointXYZRGBA>(m, "PointXYZRGBA")
      .def(nb::init())
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
      .def_rw("x", &pcl::PointXYZRGBA::x)
      .def_rw("y", &pcl::PointXYZRGBA::y)
      .def_rw("z", &pcl::PointXYZRGBA::z)
      .def("__repr__", [](const pcl::PointXYZRGBA& p) {
        return nb::str("PointXYZ(x={}, y={}, z={}, r={}, g={}, b={}, a={})").format(p.x, p.y, p.z, p.r, p.g, p.b, p.a);
      });
  
  nb::class_<pcl::PointNormal>(m, "PointXYZNormal")
      .def(nb::init())
      .def(nb::init<float, float, float>(), nb::arg("x"), nb::arg("y"), nb::arg("z"))
      .def_rw("x", &pcl::PointNormal::x)
      .def_rw("y", &pcl::PointNormal::y)
      .def_rw("z", &pcl::PointNormal::z)
      .def_rw("normal_x", &pcl::PointNormal::normal_x)
      .def_rw("normal_y", &pcl::PointNormal::normal_y)
      .def_rw("normal_z", &pcl::PointNormal::normal_z)
      .def("__repr__", [](const pcl::PointNormal& p) {
        return nb::str("PointXYZNormal(x={}, y={}, z={})").format(p.x, p.y, p.z);
      });


  // TODO must avoid code duplication, maybe write macro?
  // TODO rename Pointcloud to PointCloud to match C++ name?
  nb::class_<pcl::PointCloud<pcl::PointXYZ>>(m, "PointcloudXYZ")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointXYZ>::push_back, nb::arg("point"), "Insert a new point in the cloud, at the end of the container.")
      .def("at", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZ>::at), "Get a point from the cloud")
      .def("__getitem__",
             [](pcl::PointCloud<pcl::PointXYZ> &cloud, std::size_t i) -> pcl::PointXYZ& {
                 return cloud[i];
             }
             //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
             )
      .def("__len__", [](const pcl::PointCloud<pcl::PointXYZ> &v) { return v.size(); })
      .def("resize", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZ>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointXYZ> &v) { v.clear(); })
      .def("__iter__",
        [](const pcl::PointCloud<pcl::PointXYZ> &v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointXYZ>>(), "iterator",
                                     v.begin(), v.end());
        }, nb::keep_alive<0, 1>());

  nb::class_<pcl::PointCloud<pcl::PointXYZRGBA>>(m, "PointcloudXYZRGBA")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointXYZRGBA>::push_back)
      .def("at",
           nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZRGBA>::at),
           "Get a point from the cloud")
      .def("__getitem__",
           [](pcl::PointCloud<pcl::PointXYZRGBA>& cloud,
              std::size_t i) -> pcl::PointXYZRGBA& {
             return cloud[i];
           }
           //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
           )
      .def("__len__", [](const pcl::PointCloud<pcl::PointXYZRGBA> &v) { return v.size(); })
      .def("resize",
           nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointXYZRGBA>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointXYZRGBA> &v) { v.clear(); })
      .def(
          "__iter__",
          [](const pcl::PointCloud<pcl::PointXYZRGBA>& v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointXYZRGBA>>(),
                                     "iterator",
                                     v.begin(),
                                     v.end());
          },
          nb::keep_alive<0, 1>());
  nb::class_<pcl::PointCloud<pcl::PointNormal>>(m, "PointcloudXYZNormal")
      .def(nb::init<>())
      .def("append", &pcl::PointCloud<pcl::PointNormal>::push_back)
      .def("at", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointNormal>::at), "Get a point from the cloud")
      .def("__getitem__",
             [](pcl::PointCloud<pcl::PointNormal> &cloud, std::size_t i) -> pcl::PointNormal& {
                 return cloud[i];
             }
             //, TODO to discuss: do we want nb::rv_policy::reference_internal here? See https://nanobind.readthedocs.io/en/latest/api_extra.html#vector-bindings
             )
      .def("__len__", [](const pcl::PointCloud<pcl::PointNormal> &v) { return v.size(); })
      .def("resize", nb::overload_cast<std::size_t>(&pcl::PointCloud<pcl::PointNormal>::resize))
      .def("clear", [](pcl::PointCloud<pcl::PointNormal> &v) { v.clear(); })
      .def("__iter__",
        [](const pcl::PointCloud<pcl::PointNormal> &v) {
            return nb::make_iterator(nb::type<pcl::PointCloud<pcl::PointNormal>>(), "iterator",
                                     v.begin(), v.end());
        }, nb::keep_alive<0, 1>());
}
