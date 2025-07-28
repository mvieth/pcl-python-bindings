#include <nanobind/nanobind.h>

#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/passthrough.h>

#include "bind_utils.hpp"

namespace nb = nanobind;
using namespace nb::literals;

template<typename PointT>
pcl::PassThrough<PointT> create_pass_through(PointT&){
  return pcl::PassThrough<PointT>();
};

class PassThrough{
  public:
    double min;
    double max;
    std::string field_name;
    bool negative;

  PassThrough() {} ;

  PassThrough(
      double min, 
      double max, 
      std::string field_name, 
      bool negative) :
        min(min), 
        max(max),
        field_name(field_name),
        negative(negative) {}
};

template<typename T>
pcl::shared_ptr<T> make_shared_of_type(T&){
  return std::make_shared<T>();
};

// Without the position-restriction there is errors during the python import
template<typename T> requires (!HasPosition<T>)
PointCloud filter_return(T& cloud, const PassThrough& parameters){
  std::cout << "Does not have position.\n"; throw 101;};
template<typename T> requires HasPosition<T>
PointCloud filter_return(T& cloud, const PassThrough& parameters){
  auto filter = create_pass_through(cloud->front());
  filter.setFilterLimits(parameters.min, parameters.max);
  filter.setFilterFieldName(parameters.field_name);
  filter.setNegative(parameters.negative);
  filter.setInputCloud(cloud);

  auto output = make_shared_of_type(*cloud);
  filter.filter(*output);
  return PointCloud(output);
};


NB_MODULE(pcl_filters_ext, m)
{
  nb::class_<PassThrough>(m, "PassThrough", 
    "PassThrough passes points in a cloud based on constraints for one particular field of the point type. Iterates through the entire input once, automatically filtering non-finite points and the points outside the interval specified by setFilterLimits(), which applies only to the field specified by setFilterFieldName().")
      .def(
        nb::init<double, 
        double, 
        std::string, 
        bool>(),  
        "min"_a = 0, 
        "max"_a = 0, 
        "field_name"_a = "",
        "negative"_a = false
      )
      .def_rw("min", &PassThrough::min)
      .def_rw("max", &PassThrough::max)
      .def_rw("field_name", &PassThrough::field_name,
        "Provide the name of the field to be used for filtering data. In conjunction with setFilterLimits(), points having values outside this interval for this field will be discarded.")
      .def_rw("negative", &PassThrough::negative,
        "Set whether the regular conditions for points filtering should apply, or the inverted conditions.")
      .def("filter", [](PassThrough& filter, PointCloud& cloud){
        return std::visit([&filter](auto& arg){
          return filter_return(arg, filter);
        }, cloud.data);
      }, "Calls the filtering method and returns the filtered dataset in output.") 
      ;

      
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
  nb::class_<pcl::PassThrough<pcl::PointXYZ>>(m, "PassThroughXYZ", "PassThrough passes points in a cloud based on constraints for one particular field of the point type. Iterates through the entire input once, automatically filtering non-finite points and the points outside the interval specified by setFilterLimits(), which applies only to the field specified by setFilterFieldName().")
      .def(nb::init<>())
      .def("setInputCloud", &pcl::PassThrough<pcl::PointXYZ>::setInputCloud, nb::arg("cloud"), "Provide the input dataset.")
      .def("setFilterLimits", &pcl::PassThrough<pcl::PointXYZ>::setFilterLimits, nb::arg("limit_min"), nb::arg("limit_max"), "Set the numerical limits for the field for filtering data. In conjunction with setFilterFieldName(), points having values outside this interval for this field will be discarded.")
      .def("setFilterFieldName", &pcl::PassThrough<pcl::PointXYZ>::setFilterFieldName, nb::arg("field_name"), "Provide the name of the field to be used for filtering data. In conjunction with setFilterLimits(), points having values outside this interval for this field will be discarded.")
      .def("setNegative", &pcl::FilterIndices<pcl::PointXYZ>::setNegative, nb::arg("negative"), "Set whether the regular conditions for points filtering should apply, or the inverted conditions.")
      .def("filter", nb::overload_cast<pcl::PointCloud<pcl::PointXYZ>&>(&pcl::Filter<pcl::PointXYZ>::filter), nb::arg("output"), "Calls the filtering method and returns the filtered dataset in output.") // TODO alternatively could return output, not as parameter?
      ;

  nb::class_<pcl::PassThrough<pcl::PointXYZRGBA>>(m, "PassThroughXYZRGBA")
      .def(nb::init<>())
      .def("setInputCloud", &pcl::PassThrough<pcl::PointXYZRGBA>::setInputCloud)
      .def("setFilterLimits", &pcl::PassThrough<pcl::PointXYZRGBA>::setFilterLimits)
      .def("setFilterFieldName",
           &pcl::PassThrough<pcl::PointXYZRGBA>::setFilterFieldName)
      .def("setNegative",
           &pcl::FilterIndices<pcl::PointXYZRGBA>::setNegative)
      .def("filter",
           nb::overload_cast<pcl::PointCloud<pcl::PointXYZRGBA>&>(
               &pcl::Filter<pcl::PointXYZRGBA>::filter))
      ;
}
