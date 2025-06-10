#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

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

  m.def("loadPCDFile", &pcl::io::loadPCDFile<pcl::PointXYZ>);
  m.def("loadPCDFile", &pcl::io::loadPCDFile<pcl::PointXYZRGBA>);
  m.def("loadPCDFile", &pcl::io::loadPCDFile<pcl::PointNormal>);
  m.def("loadPLYFile", &pcl::io::loadPLYFile<pcl::PointXYZ>);
  m.def("loadPLYFile", &pcl::io::loadPLYFile<pcl::PointXYZRGBA>);
  m.def("savePCDFileBinary", &pcl::io::savePCDFileBinary<pcl::PointXYZ>);
  m.def("savePCDFileBinary", &pcl::io::savePCDFileBinary<pcl::PointXYZRGBA>);
}
