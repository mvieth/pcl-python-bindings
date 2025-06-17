# for pytest
import pytest
from pcl.common import PointcloudXYZ, PointXYZ
from pcl.registration import IterativeClosestPointXYZ

def test_iterativeclosestpoint():
    cloud_source = PointcloudXYZ()
    cloud_source.append(PointXYZ(0, 0, 0))
    cloud_source.append(PointXYZ(0, 10, 0))
    cloud_source.append(PointXYZ(0, 0, 10))
    cloud_target = PointcloudXYZ()
    cloud_target.append(PointXYZ(1, 0, 0))
    cloud_target.append(PointXYZ(1, 10, 0))
    cloud_target.append(PointXYZ(1, 0, 10))
    icp = IterativeClosestPointXYZ()
    icp.setInputSource(cloud_source)
    icp.setInputTarget(cloud_target)
    cloud_source_aligned = PointcloudXYZ()
    icp.align(cloud_source_aligned)
    assert len(cloud_source_aligned)==3
    assert icp.hasConverged()
    transf = icp.getFinalTransformation()
    assert transf[0, 3] == pytest.approx(1.0)
