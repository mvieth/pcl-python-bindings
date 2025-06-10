# for pytest
from pcl.common import PointcloudXYZ, PointXYZ
from pcl.filters import PassThroughXYZ

def test_passthrough():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 0.0, 0.0))
    cloud.append(PointXYZ(0.0, 0.0, 1.0))
    cloud.append(PointXYZ(0.0, 0.0, 2.0))
    filter = PassThroughXYZ()
    filter.setInputCloud(cloud)
    filter.setFilterFieldName("z")
    filter.setFilterLimits(0.5, 1.5)
    cloud_out = PointcloudXYZ()
    filter.filter(cloud_out)
    assert len(cloud_out) == 1
    assert cloud_out[0].z == 1.0
