# Experimental python bindings

1. Install PCL (vcpkg or build from https://github.com/PointCloudLibrary/pcl or whatever)
2. Clone this repository and `cd` into it
3. Run `pip install .`
4. Optionally run `pytest`

Example:

```python
>>> from pcl.common import PointcloudXYZ, PointXYZ
>>> from pcl.filters import PassThroughXYZ
>>> cloud = PointcloudXYZ()
>>> cloud.append(PointXYZ(0.0, 0.0, 0.0))
>>> cloud.append(PointXYZ(0.0, 0.0, 1.0))
>>> cloud.append(PointXYZ(0.0, 0.0, 2.0))
>>> filter = PassThroughXYZ()
>>> filter.setInputCloud(cloud)
>>> filter.setFilterFieldName("z")
>>> filter.setFilterLimits(0.5, 1.5)
>>> cloud_out = PointcloudXYZ()
>>> filter.filter(cloud_out)
>>> len(cloud_out)
1
>>> cloud_out[0]
PointXYZ(x=0.0, y=0.0, z=1.0)
```

nanobind_example
================

|      CI              | status |
|----------------------|--------|
| pip builds           | [![Pip Action Status][actions-pip-badge]][actions-pip-link] |
| wheels               | [![Wheel Action Status][actions-wheels-badge]][actions-wheels-link] |

[actions-pip-link]:        https://github.com/wjakob/nanobind_example/actions?query=workflow%3APip
[actions-pip-badge]:       https://github.com/wjakob/nanobind_example/workflows/Pip/badge.svg
[actions-wheels-link]:     https://github.com/wjakob/nanobind_example/actions?query=workflow%3AWheels
[actions-wheels-badge]:    https://github.com/wjakob/nanobind_example/workflows/Wheels/badge.svg


This repository contains a tiny project showing how to create C++ bindings
using [nanobind](https://github.com/wjakob/nanobind) and
[scikit-build-core](https://scikit-build-core.readthedocs.io/en/latest/index.html). It
was derived from the corresponding _pybind11_ [example
project](https://github.com/pybind/scikit_build_example/) developed by
[@henryiii](https://github.com/henryiii).

Furthermore, the [bazel](https://github.com/wjakob/nanobind_example/tree/bazel) branch contains an example
on how to build nanobind bindings extensions with Bazel using the [nanobind-bazel](https://github.com/nicholasjng/nanobind-bazel/) project.

Installation
------------

1. Clone this repository
2. Run `pip install ./nanobind_example`

Afterwards, you should be able to issue the following commands (shown in an
interactive Python session):

```pycon
>>> import nanobind_example
>>> nanobind_example.add(1, 2)
3
```

CI Examples
-----------

The `.github/workflows` directory contains two continuous integration workflows
for GitHub Actions. The first one (`pip`) runs automatically after each commit
and ensures that packages can be built successfully and that tests pass.

The `wheels` workflow uses
[cibuildwheel](https://cibuildwheel.readthedocs.io/en/stable/) to automatically
produce binary wheels for a large variety of platforms. If a `pypi_password`
token is provided using GitHub Action's _secrets_ feature, this workflow can
even automatically upload packages on PyPI.


License
-------

_nanobind_ and this example repository are both provided under a BSD-style
license that can be found in the [LICENSE](./LICENSE) file. By using,
distributing, or contributing to this project, you agree to the terms and
conditions of this license.
