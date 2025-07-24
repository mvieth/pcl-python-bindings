#include <concepts>
#include <string>
#include <variant>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace nb = nanobind;

using NumpyArray2d = nb::ndarray<float, nb::numpy, nb::ndim<2>>;
using InputArray2d = nb::ndarray<float, nb::ndim<2>>;

using CloudVariant = std::variant<
  pcl::PointCloud<pcl::PointXYZ>,
  pcl::PointCloud<pcl::PointXYZRGBA>,
  pcl::PointCloud<pcl::PointNormal>,
  pcl::PointCloud<pcl::Normal>
>;

template<typename T>
inline nb::capsule delete_owner(T data){
  // Delete 'data' when the 'owner' capsule expires
  nb::capsule owner(data, [](void *p) noexcept {delete[] (float *) p;});
  return owner;
};

// Make sure enum has same order as cloud variant for variant.index()
enum PointType{
  PointXYZ,
  PointXYZRGBA,
  PointNormal,
  Normal 
}; 

template<typename T>
concept HasPosition = requires (T cloud){
  cloud.front().x;
  cloud.front().y;
  cloud.front().z;
};

template<typename T>
concept HasNormal= requires (T cloud){
  cloud.front().normal_x;
  cloud.front().normal_y;
  cloud.front().normal_z;
};


template<typename T> requires (!HasPosition<T>)
void set_positions(T &cloud, InputArray2d array)  {std::cout << "Key does not exist.\n"; throw 101;};
template<typename T> requires HasPosition<T>
void set_positions(T &cloud, InputArray2d array)
{
  cloud.resize(array.shape(0));

  for (int ii = 0; ii < array.shape(0); ii++){
    cloud[ii].x = array(ii, 0);
    cloud[ii].y = array(ii, 1);
    cloud[ii].z = array(ii, 2);
  }
};

template<typename T> requires (!HasPosition<T>)
NumpyArray2d get_positions(T &cloud) {std::cout << "Key does not exist.\n"; throw 101;};
template<typename T> requires HasPosition<T>
NumpyArray2d get_positions(T &cloud){
  char cols = 3;
  auto rows = cloud.size(); 

  float *data = new float[rows * cols];
  for (size_t i = 0; i < rows; ++i){
    auto value = cloud[i];
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
  cloud.resize(array.shape(0));

  for (int ii = 0; ii < array.shape(0); ii++){
    cloud[ii].normal_x = array(ii, 0);
    cloud[ii].normal_y = array(ii, 1);
    cloud[ii].normal_z = array(ii, 2);
  }
};

template<typename T> requires (!HasNormal<T>)
NumpyArray2d get_normals(T &cloud) {std::cout << "Key does not exist.\n"; throw 101;};
template<typename T> requires HasNormal<T>
NumpyArray2d get_normals(T &cloud){
  char cols = 3;
  auto rows = cloud.size(); 

  float *data = new float[rows * cols];
  for (size_t i = 0; i < rows; ++i){
    auto value = cloud[i];
    data[i * cols]  = value.normal_x;
    data[i * cols + 1]  = value.normal_y;
    data[i * cols + 2]  = value.normal_z;
  }

  auto owner = delete_owner(data);

  return NumpyArray2d(data,{ rows, cols }, owner);
};


class PointCloud
{
  public:
    CloudVariant data;

  PointCloud(CloudVariant data_): data(data_) {}

  PointCloud(PointType type) {
    switch(type) {
      case PointXYZ: {
        data = pcl::PointCloud<pcl::PointXYZ>();
        break;
      }
      case PointNormal: {
        data = pcl::PointCloud<pcl::PointNormal>();
        break;
      }
      case Normal: {
        data = pcl::PointCloud<pcl::Normal>();
        break;
      }
      default:
      {}
        // code block
    }
  }

  size_t size() const {return std::visit([](auto&& arg){ return arg.size();}, data);}
  PointType type() const {return static_cast<PointType>(data.index());}
};

namespace keys {
  using KeyList = std::vector<std::string>;

  std::string position = "position";
  std::string normal = "normal";
  std::string curvature = "curvature";

  template<typename T> requires (!HasPosition<T>)
  void add_position_key(T&, KeyList&) {return;};
  template<typename T> requires HasPosition<T>
  void add_position_key(T&, KeyList& list){list.push_back(position);}

  template<typename T> requires (!HasNormal<T>)
  void add_normal_key(T&, KeyList&) {return;};
  template<typename T> requires HasNormal<T>
  void add_normal_key(T&, KeyList& list) {list.push_back(normal);}

  KeyList get_keys (const PointCloud& cloud) {
    return std::visit([](auto&& arg){
      KeyList keys = {};
      add_position_key(arg, keys);
      add_normal_key(arg, keys);
      return keys;
    }, cloud.data);
  }
}
