#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include "bind_utils.hpp"

namespace nb = nanobind;
using namespace nb::literals;

class IterativeClosestPoint{
  public:
    double inlier_threshold; // = 0.05
    unsigned int max_iterations; // = 10
    unsigned int ransac_iterations; // = 0 
    double euclidean_fitness_epsilon_ = std::numeric_limits<double>::max();
    // TODO: more settings / tweaking of algorithm
    // unsigned int min_number_correspondences; // = 3 ? -> no setter in class
    // setTransformationEpsilon
    // setMaxCorrespondenceDistance (0.05);

    // Result
    bool converged_ = false;
    Eigen::Matrix4f final_transformation_ = Eigen::Matrix4f::Identity();

    IterativeClosestPoint(
      double inlier_threshold, 
      unsigned int max_iterations,
      unsigned int ransac_iterations
    ): 
        inlier_threshold(inlier_threshold), 
        max_iterations(max_iterations), 
        ransac_iterations(ransac_iterations) {}
};

template<typename PointT, typename PointS> 
pcl::IterativeClosestPoint<PointT, PointS, float> 
create_icp(PointT&, PointS){
  // All pcl-types seem to be points
  return pcl::IterativeClosestPoint<PointT, PointS, float>();
};


template<typename CloudT, typename CloudS> requires (!HasPosition<CloudT> || !HasPosition<CloudS>)
void align(CloudT& source,  CloudS& target, IterativeClosestPoint& parameters){
std::cout << "Does not have position.\n"; throw 101;};
template<typename CloudT, typename CloudS> requires (HasPosition<CloudT> && HasPosition<CloudS>)
void align(CloudT& source,  CloudS& target, IterativeClosestPoint& parameters){
  auto icp = create_icp(source->front(), target->front());

  icp.setRANSACOutlierRejectionThreshold(parameters.inlier_threshold);
  icp.setMaximumIterations(parameters.max_iterations);
  icp.setRANSACIterations(parameters.ransac_iterations);

  icp.setInputSource(source);
  icp.setInputTarget(target);

  auto cloud_source_aligned = make_shared_of_type(*source);
  icp.align(*cloud_source_aligned);

  parameters.converged_ = icp.hasConverged();
  if (!parameters.converged_) return;

  parameters.euclidean_fitness_epsilon_ = icp.getEuclideanFitnessEpsilon();
  parameters.final_transformation_ = icp.getFinalTransformation(); 
};

NB_MODULE(pcl_registration_ext, m)
{
  nb::class_<IterativeClosestPoint>(m, "IterativeClosestPoint")
  .def(nb::init<double, unsigned int, unsigned int>(),
    "inlier_threshold"_a = 0.05, "max_iterations"_a  = 10, "ransac_iterations"_a = 0)
  .def_rw("inlier_threshold", &IterativeClosestPoint::inlier_threshold)
  .def_rw("max_iterations", &IterativeClosestPoint::max_iterations)
  .def_rw("ransac_iterations", &IterativeClosestPoint::ransac_iterations)
  .def_ro("euclidean_fitness_epsilon_", &IterativeClosestPoint::euclidean_fitness_epsilon_)
  .def_ro("converged_", &IterativeClosestPoint::converged_, 
    "Return the state of convergence after the last align run.")
  .def_ro("final_transformation_", &IterativeClosestPoint::final_transformation_, 
    "Get the final transformation matrix estimated by the registration method.")
  .def("align", [](IterativeClosestPoint& icp, PointCloud& source, PointCloud& target){
    std::visit([&icp](auto& source_, auto& target_){
      align(source_, target_, icp);
    }, source.data, target.data);
  }, "source"_a, "target"_a,
  "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.") 
  ;

  nb::class_<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>>(m, "IterativeClosestPointXYZ") // TODO to discuss: Scalar=float or =double?
  .def(nb::init<>())
  .def("getFinalTransformation", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::getFinalTransformation, "Get the final transformation matrix estimated by the registration method.")
  .def("hasConverged", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::hasConverged, "Return the state of convergence after the last align run.")
  .def("setInputSource", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::setInputSource, nb::arg("cloud"), "Provide a pointer to the input source (e.g., the point cloud that we want to align to the target)")
  .def("setInputTarget", &pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::setInputTarget, nb::arg("cloud"), "Provide a pointer to the input target (e.g., the point cloud that we want to align the input source to)")
  // .def("align", 
  //   nb::overload_cast
  //   <pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::PointCloudSource&>
  //   (&pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::align),
  //               nb::arg("output"), "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.") // TODO could alternatively return output?
  //.def("align", nb::overload_cast<pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::PointCloudSource&, const pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4&>(&pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::align), nb::arg("output"), nb::arg("guess")=pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ, float>::Matrix4::Identity(), "Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.")
  ;
}
