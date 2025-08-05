#include <concepts>
#include <memory>
#include <string>
#include <utility>
#include <variant>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>

#include <pcl/point_types.h> 
#include <pcl/point_cloud.h>

namespace nb = nanobind;

using NumpyArray2d = nb::ndarray<float, nb::numpy, nb::ndim<2>>;
using InputArray2d = nb::ndarray<float, nb::ndim<2>>;

template<typename T>
inline nb::capsule delete_owner(T data){
  // Delete 'data' when the 'owner' capsule expires
  nb::capsule owner(data, [](void *p) noexcept {delete[] (float *) p;});
  return owner;
};

template<typename T>
pcl::shared_ptr<T> make_shared_of_type(const T&){
  return std::make_shared<T>();
};


using CloudVariant = std::variant<
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBA>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBL>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZLAB>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZHSV>>,
  std::shared_ptr<pcl::PointCloud<pcl::Normal>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointNormal>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGBNormal>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZINormal>>,
  std::shared_ptr<pcl::PointCloud<pcl::PointXYZLNormal>>
>;

// Make sure enum matches the cloud variant to enable variant.index() / variant.emplace()
enum PointType{
  PointXYZ,
  PointXYZI,
  PointXYZL,
  PointXYZRGBA,
  PointXYZRGB,
  PointXYZRGBL,
  PointXYZLAB,
  PointXYZHSV,
  Normal,
  PointNormal,
  PointXYZRGBNormal,
  PointXYZINormal,
  PointXYZLNormal,
  NUMBER_OF_POINTS // The number of enum-points for correct templatization 
};


// namespace traits {
  template<typename T>
  concept HasPosition = requires (T cloud){
    cloud->front().x;
    cloud->front().y;
    cloud->front().z;
  };

  template<typename T>
  concept HasNormal = requires (T cloud){
    cloud->front().normal_x;
    cloud->front().normal_y;
    cloud->front().normal_z;
  };

  template<typename T>
  concept HasCurvature = requires (T cloud){
    cloud->front().curvature;
  };

  template<typename T>
  concept HasColor = requires (T cloud){
    cloud->front().r;
    cloud->front().g;
    cloud->front().b;
    cloud->front().a;
  };

  template<typename T>
  concept HasHSV = requires (T cloud){
    cloud->front().h;
    cloud->front().s;
    cloud->front().v;
  };

  template<typename T>
  concept HasLabel = requires (T cloud){
    cloud->front().label;
  };

  template<typename T>
  concept HasIntensity = requires (T cloud){
    cloud->front().intensity;
  };

  template<typename T>
  concept HasLab = requires (T cloud){
    cloud->front().L;
    cloud->front().a;
    cloud->front().b;
  };



// } // namespace traits


template<size_t CloudT>
CloudVariant makeCloudForIndex() {
  CloudVariant tmp;
  tmp.emplace<CloudT>();
  CloudVariant cloud = make_shared_of_type(*std::get<CloudT>(tmp));
  return cloud;
};

class PointCloud
{
  template<size_t... Is>
  CloudVariant makeCloud(size_t nIdx, std::index_sequence<Is...>) {
    /* In order to work with std::variant-templatization: create all 
    function-pointers, and call the correct one.  */
    using FuncType = CloudVariant(*)();
    constexpr FuncType arFuncs[] = { makeCloudForIndex<Is>... };
    return arFuncs[nIdx]();
  }

  public:
     CloudVariant data;

    PointCloud(CloudVariant data_): data(data_) {}

    PointCloud(PointType type) {
      data = makeCloud(static_cast<size_t>(type), std::make_index_sequence<PointType::NUMBER_OF_POINTS>());
    }

    size_t size() const {
      return std::visit([](auto&& arg){return arg->size();}, data);
    }
    PointType type() const {return static_cast<PointType>(data.index());}
};

namespace keys {
  using KeyList = std::vector<std::string>;

  std::string position = "position";
  std::string normal = "normal";
  std::string curvature = "curvature";
  std::string color = "color";
  std::string intensity = "intensity";
  std::string label = "label";
  std::string hsv = "hsv";
  std::string lab = "Lab";

  template<typename T> requires (!HasPosition<T>) void add_position(T&, KeyList&) {};
  template<typename T> requires HasPosition<T> void add_position(T&, KeyList& list){list.push_back(position);}

  template<typename T> requires (!HasNormal<T>) void add_normal(T&, KeyList&) {};
  template<typename T> requires HasNormal<T> void add_normal(T&, KeyList& list) {list.push_back(normal);}

  template<typename T> requires (!HasCurvature<T>) void add_curvature(T&, KeyList&) {};
  template<typename T> requires HasCurvature<T> void add_curvature(T&, KeyList& list) {list.push_back(curvature);}

  template<typename T> requires (!HasColor<T>) void add_color(T&, KeyList&) {};
  template<typename T> requires HasColor<T> void add_color(T&, KeyList& list) {list.push_back(color);}

  template<typename T> requires (!HasIntensity<T>) void add_intensity(T&, KeyList&) {};
  template<typename T> requires HasIntensity<T> void add_intensity(T&, KeyList& list) {list.push_back(intensity);}

  template<typename T> requires (!HasLabel<T>) void add_label(T&, KeyList&) {};
  template<typename T> requires HasLabel<T> void add_label(T&, KeyList& list) {list.push_back(label);}

  template<typename T> requires (!HasHSV<T>) void add_hsv(T&, KeyList&) {};
  template<typename T> requires HasHSV<T> void add_hsv(T&, KeyList& list) {list.push_back(hsv);}

  template<typename T> requires (!HasLab<T>) void add_lab(T&, KeyList&) {};
  template<typename T> requires HasLab<T> void add_lab(T&, KeyList& list) {list.push_back(lab);}

  KeyList get_keys (const PointCloud& cloud) {
    return std::visit([](auto&& arg){
      KeyList keys = {};
      add_position(arg, keys);
      add_normal(arg, keys);
      add_curvature(arg, keys);
      add_color(arg, keys);
      add_intensity(arg, keys);
      add_label(arg, keys);
      add_hsv(arg, keys);
      add_lab(arg, keys);
      return keys;
    }, cloud.data);
  }
}
