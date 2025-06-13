#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h> // for loadPLYFile
#include <pcl/io/auto_io.h> // for load

#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

namespace nb = nanobind;

NB_MODULE(pcl_io_ext, m)
{
  m.def("ReadCloudXYZ", [](const std::string& filePath) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCDReader p;
        p.read(filePath, cloud);
        return cloud;
      },
      "Read a PCD file containing PointXYZ data",
      nb::arg("filePath"));

  m.def("loadPCDFile", &pcl::io::loadPCDFile<pcl::PointXYZ>, nb::arg("file_name"), nb::arg("cloud"), "Load any PCD file into a templated PointCloud type.");
  m.def("loadPCDFile", &pcl::io::loadPCDFile<pcl::PointXYZRGBA>, nb::arg("file_name"), nb::arg("cloud"), "Load any PCD file into a templated PointCloud type.");
  m.def("loadPCDFile", &pcl::io::loadPCDFile<pcl::PointNormal>, nb::arg("file_name"), nb::arg("cloud"), "Load any PCD file into a templated PointCloud type.");
  m.def("loadPLYFile", &pcl::io::loadPLYFile<pcl::PointXYZ>, nb::arg("file_name"), nb::arg("cloud"), "Load any PLY file into a templated PointCloud type.");
  m.def("loadPLYFile", &pcl::io::loadPLYFile<pcl::PointXYZRGBA>, nb::arg("file_name"), nb::arg("cloud"), "Load any PLY file into a templated PointCloud type.");
  m.def("load", &pcl::io::load<pcl::PointXYZ>, nb::arg("file_name"), nb::arg("cloud"), "Load a file into a template PointCloud type according to extension.");
  m.def("load", &pcl::io::load<pcl::PointXYZRGBA>, nb::arg("file_name"), nb::arg("cloud"), "Load a file into a template PointCloud type according to extension.");
  m.def("load", &pcl::io::load<pcl::PointNormal>, nb::arg("file_name"), nb::arg("cloud"), "Load a file into a template PointCloud type according to extension.");
  m.def("savePCDFileBinary", &pcl::io::savePCDFileBinary<pcl::PointXYZ>, nb::arg("file_name"), nb::arg("cloud"), "Templated version for saving point cloud data to a PCD file containing a specific given cloud format. The resulting file will be an uncompressed binary.");
  m.def("savePCDFileBinary", &pcl::io::savePCDFileBinary<pcl::PointXYZRGBA>, nb::arg("file_name"), nb::arg("cloud"), "Templated version for saving point cloud data to a PCD file containing a specific given cloud format. The resulting file will be an uncompressed binary.");
}
