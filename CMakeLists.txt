cmake_minimum_required(VERSION 3.15...3.26)

project(nanobind_example LANGUAGES C CXX)

if (NOT SKBUILD)
  message(WARNING "\
  This CMake file is meant to be executed using 'scikit-build'. Running
  it directly will almost certainly not produce the desired result. If
  you are a user trying to install this package, please use the command
  below, which will install all necessary build dependencies, compile
  the package in an isolated environment, and then install it.
  =====================================================================
   $ pip install .
  =====================================================================
  If you are a software developer, and this is your own package, then
  it is usually much more efficient to install the build dependencies
  in your environment once and use the following command that avoids
  a costly creation of a new virtual environment at every compilation:
  =====================================================================
   $ pip install nanobind scikit-build-core[pyproject]
   $ pip install --no-build-isolation -ve .
  =====================================================================
  You may optionally add -Ceditable.rebuild=true to auto-rebuild when
  the package is imported. Otherwise, you need to re-run the above
  after editing C++ files.")
endif()

find_package(PCL REQUIRED)

# Try to import all Python components potentially needed by nanobind
find_package(Python 3.8
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

add_compile_options(-fconcepts)

# We are now ready to compile the actual extension module
#nanobind_add_module(
  # Name of the extension
#  nanobind_example_ext

  # Target the stable ABI for Python 3.12+, which reduces
  # the number of binary wheels that must be built. This
  # does nothing on older Python versions
#  STABLE_ABI

  # Build libnanobind statically and merge it into the
  # extension (which itself remains a shared library)
  #
  # If your project builds multiple extensions, you can
  # replace this flag by NB_SHARED to conserve space by
  # reusing a shared libnanobind across libraries
#  NB_STATIC

  # Source code goes here
#  src/nanobind_example_ext.cpp
#)

nanobind_add_module(pcl_common_ext src/python_common.cpp)
target_link_libraries(pcl_common_ext PRIVATE pcl_common)

nanobind_add_module(pcl_io_ext src/python_io.cpp)
target_link_libraries(pcl_io_ext PRIVATE pcl_io)

nanobind_add_module(pcl_features_ext src/python_features.cpp)
target_link_libraries(pcl_features_ext PRIVATE pcl_features)

nanobind_add_module(pcl_filters_ext src/python_filters.cpp)
target_link_libraries(pcl_filters_ext PRIVATE pcl_filters)

nanobind_add_module(pcl_registration_ext src/python_registration.cpp)
target_link_libraries(pcl_registration_ext PRIVATE pcl_registration)

nanobind_add_module(pcl_segmentation_ext src/python_segmentation.cpp)
target_link_libraries(pcl_segmentation_ext PRIVATE pcl_segmentation)

nanobind_add_module(pcl_visualization_ext src/python_visualization.cpp)
target_link_libraries(pcl_visualization_ext PRIVATE pcl_visualization)

# Install directive for scikit-build-core
install(TARGETS pcl_common_ext LIBRARY DESTINATION pcl/common)
install(TARGETS pcl_io_ext LIBRARY DESTINATION pcl/io)
install(TARGETS pcl_features_ext LIBRARY DESTINATION pcl/features)
install(TARGETS pcl_filters_ext LIBRARY DESTINATION pcl/filters)
install(TARGETS pcl_registration_ext LIBRARY DESTINATION pcl/registration)
install(TARGETS pcl_segmentation_ext LIBRARY DESTINATION pcl/segmentation)
install(TARGETS pcl_visualization_ext LIBRARY DESTINATION pcl/visualization)
