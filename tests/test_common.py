# for pytest
from pcl.common import PointcloudXYZ, PointXYZ

def test_pointcloud():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 1.0, 0.0))
    assert cloud[0].y == 1.0
    cloud.resize(5)
    assert len(cloud) == 5
    #cloud[0] = PointXYZ(0.0, 1.0, 0.0)
    # TODO expand
