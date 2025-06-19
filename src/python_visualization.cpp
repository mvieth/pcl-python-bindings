#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>

namespace nb = nanobind;

NB_MODULE(pcl_visualization_ext, m)
{
  nb::class_<pcl::visualization::PCLVisualizer>(m, "PCLVisualizer")
      .def(nb::init<>())
      /*.def("AddPointCloudXYZ",
             nb::overload_cast<const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&,
                               const std::string&,
                               int>(&pcl::visualization::PCLVisualizer::addPointCloud),
             "Add point cloud to visualizer.")*/
      .def("AddPointCloud", 
           nb::overload_cast<const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&,
                             const std::string&,
                             int>(
           &pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZ>),
           nb::arg("cloud"), nb::arg("id")="cloud", nb::arg("viewport")=0, "Add point cloud.")
      .def("AddPointCloud", 
           nb::overload_cast<const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&,
                             const std::string&,
                             int>(
               &pcl::visualization::PCLVisualizer::addPointCloud<pcl::PointXYZRGBA>),
           nb::arg("cloud"), nb::arg("id")="cloud", nb::arg("viewport")=0, "Add point cloud.")
      .def("removeAllShapes", &pcl::visualization::PCLVisualizer::removeAllShapes, nb::arg("viewport")=0, "Clear the visualizer.")
      .def("spinOnce", &pcl::visualization::PCLVisualizer::spinOnce, nb::arg("time")=1, nb::arg("force_redraw")=false, "Runs the visualizer loop once")
      .def("addCoordinateSystem",
           nb::overload_cast<double, const std::string&, int>(&pcl::visualization::PCLVisualizer::addCoordinateSystem),
           nb::arg("scale")=1.0, nb::arg("id")="reference", nb::arg("viewport")=0, "Adds a coordinate system")
    ;
}
