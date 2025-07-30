#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h> // TODO alternatively nb::bind_vector?

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/extract_clusters.h>

#include "bind_utils.hpp"

namespace nb = nanobind;
using namespace nb::literals;


class EuclideanClusterExtraction{
  public:
    double tolerance;
    unsigned int min_cluster_size;
    unsigned int max_cluster_size;

  EuclideanClusterExtraction(
    double tolerance,
    unsigned int min_cluster_size,
    unsigned int max_cluster_size
  ) :
    tolerance(tolerance),
    min_cluster_size(min_cluster_size),
    max_cluster_size(max_cluster_size)  {}
};


template<typename PointT> 
pcl::EuclideanClusterExtraction<PointT> create_clusterer(PointT&){
  return pcl::EuclideanClusterExtraction<PointT>();
};

template<typename CloudT> requires (!HasPosition<CloudT>)
std::vector<pcl::Indices> extract(CloudT&, EuclideanClusterExtraction&){
  std::cout << "Does not have position.\n"; throw 101;};
template<typename CloudT> requires HasPosition<CloudT>
std::vector<pcl::Indices> extract(CloudT& cloud, EuclideanClusterExtraction& parameters){
  auto ece = create_clusterer(cloud->front());

  ece.setClusterTolerance(parameters.tolerance);
  ece.setMinClusterSize(parameters.min_cluster_size);
  ece.setMaxClusterSize(parameters.max_cluster_size);
  ece.setInputCloud(cloud);

  std::vector< pcl::PointIndices > clusters2;
  ece.extract(clusters2);
  std::vector<pcl::Indices> clusters;
  clusters.reserve(clusters2.size());

  for(const auto& cluster: clusters2) { clusters.push_back(cluster.indices); }

  return clusters;
};



NB_MODULE(pcl_segmentation_ext, m)
{
  nb::class_<EuclideanClusterExtraction>(m, "EuclideanClusterExtraction", 
    "EuclideanClusterExtraction represents a segmentation class for cluster extraction in an Euclidean sense.")
  .def(nb::init<double, unsigned int, unsigned int>(), 
    "tolerance"_a=0.0, 
    "min_cluster_size"_a=1,
    "max_cluster_size"_a=((std::numeric_limits<int>::max)()) 
  )
  .def_rw("tolerance", &EuclideanClusterExtraction::tolerance)
  .def_rw("min_cluster_size", &EuclideanClusterExtraction::min_cluster_size)
  .def_rw("max_cluster_size", &EuclideanClusterExtraction::max_cluster_size)
  .def("extract", [](EuclideanClusterExtraction& ece, PointCloud& cloud){
    return std::visit([&ece](auto& arg){
      return extract(arg, ece);
    }, cloud.data);
  })
  ;


  nb::class_<pcl::EuclideanClusterExtraction<pcl::PointXYZ>>(m, "EuclideanClusterExtractionXYZ", "EuclideanClusterExtraction represents a segmentation class for cluster extraction in an Euclidean sense.")
      .def(nb::init<>())
      .def("setInputCloud", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setInputCloud, nb::arg("cloud")) // TODO maybe inherit this from PCLBase?
      .def("setClusterTolerance", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setClusterTolerance, nb::arg("tolerance"), "Set the spatial cluster tolerance as a measure in the L2 Euclidean space.")
      .def("setMinClusterSize", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setMinClusterSize, nb::arg("min_cluster_size"), "Set the minimum number of points that a cluster needs to contain in order to be considered valid.")
      .def("setMaxClusterSize", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::setMaxClusterSize, nb::arg("max_cluster_size"), "Set the maximum number of points that a cluster needs to contain in order to be considered valid.")
      //.def("extract", &pcl::EuclideanClusterExtraction<pcl::PointXYZ>::extract, nb::arg("clusters"), "Cluster extraction in a PointCloud.")
      .def("extract", [](pcl::EuclideanClusterExtraction<pcl::PointXYZ>& ece){
        std::vector< pcl::PointIndices > clusters2;
        ece.extract(clusters2);
        std::vector<pcl::Indices> clusters;
        clusters.reserve(clusters2.size());
        for(const auto& cluster: clusters2) { clusters.push_back(cluster.indices); }
        return clusters;
      }, "Cluster extraction in a PointCloud. Returns a list of lists of indices, so for example clusters[0] is the first cluster, and clusters[0][0] is the index of the first point in the first cluster.")
      ;
}
