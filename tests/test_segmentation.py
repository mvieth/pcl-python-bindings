# for pytest
from pcl.common import PointcloudXYZ, PointXYZ
from pcl.segmentation import EuclideanClusterExtractionXYZ

def test_euclideanclusterextraction():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 0.0, 0.0))
    cloud.append(PointXYZ(0.1, 0.0, 0.0))
    cloud.append(PointXYZ(2.0, 0.0, 0.0))
    cloud.append(PointXYZ(2.1, 0.0, 0.0))
    seg = EuclideanClusterExtractionXYZ()
    seg.setInputCloud(cloud)
    seg.setClusterTolerance(1.0)
    clusters = seg.extract()
    assert len(clusters[0]) == 2
    assert len(clusters[1]) == 2
    assert clusters[0][0] == 0
    assert clusters[0][1] == 1
    assert clusters[1][0] == 2
    assert clusters[1][1] == 3
