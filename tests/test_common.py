# for pytest
import numpy as np
from pcl.common import PointcloudXYZ, PointXYZ, PointCloud
from pcl.common import PointType as tt


def test_pointcloud():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 1.0, 0.0))
    assert cloud[0].y == 1.0
    cloud.resize(5)
    assert len(cloud) == 5
    # cloud[0] = PointXYZ(0.0, 1.0, 0.0)
    # TODO expand


def test_general_cloud():
    cloud = PointCloud(tt.PointXYZ)
    assert len(cloud) == 0
    assert "position" in cloud.keys()
    assert "normal" not in cloud.keys()
    position = np.array([[1, 2, 3], [4, 5, 6]])
    cloud["position"] = position
    result = cloud["position"] 
    np.testing.assert_allclose(position, result)
    assert len(cloud) == position.shape[0]

    cloud = PointCloud(tt.Normal)
    assert "position" not in cloud.keys()
    assert "normal" in cloud.keys()
    normal = np.array([[1, 2, 3], [4, 5, 6]])
    cloud["normal"] = normal
    result = cloud["normal"] 
    np.testing.assert_allclose(normal, result)
    assert len(cloud) == normal.shape[0]

def test_feature():
    pass


if __name__ == "__main__":
    test_general_cloud()
