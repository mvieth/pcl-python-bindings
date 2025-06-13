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

Build

|      CI              | status |
|----------------------|--------|
| pip builds           | [![Pip Action Status][actions-pip-badge]][actions-pip-link] |
| wheels               | [![Wheel Action Status][actions-wheels-badge]][actions-wheels-link] |

[actions-pip-link]:        https://github.com/mvieth/pcl-python-bindings/actions?query=workflow%3APip
[actions-pip-badge]:       https://github.com/mvieth/pcl-python-bindings/workflows/Pip/badge.svg
[actions-wheels-link]:     https://github.com/mvieth/pcl-python-bindings/actions?query=workflow%3AWheels
[actions-wheels-badge]:    https://github.com/mvieth/pcl-python-bindings/workflows/Wheels/badge.svg

