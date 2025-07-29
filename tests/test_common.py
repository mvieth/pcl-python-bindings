# for pytest
from time import perf_counter

import numpy as np
from pcl.common import PointcloudXYZ, PointXYZ, PointCloud
from pcl.common import PointType as t


def test_pointcloud():
    cloud = PointcloudXYZ()
    cloud.append(PointXYZ(0.0, 1.0, 0.0))
    assert cloud[0].y == 1.0
    cloud.resize(5)
    assert len(cloud) == 5
    # cloud[0] = PointXYZ(0.0, 1.0, 0.0)
    # TODO expand


def test_general_cloud():
    cloud = PointCloud(t.PointXYZ)
    assert len(cloud) == 0
    assert "position" in cloud.keys()
    assert "normal" not in cloud.keys()
    position = np.array([[1, 2, 3], [4, 5, 6]])
    cloud["position"] = position
    result = cloud["position"]
    np.testing.assert_allclose(position, result)
    assert len(cloud) == position.shape[0]

    cloud = PointCloud(t.Normal)
    assert "position" not in cloud.keys()
    assert "normal" in cloud.keys()
    normal = np.array([[1, 2, 3], [4, 5, 6]])
    cloud["normal"] = normal
    result = cloud["normal"]
    np.testing.assert_allclose(normal, result)
    assert len(cloud) == normal.shape[0]


def everything_is_point(n_points: int):
    cloud = PointcloudXYZ()
    cloud.resize(n_points)

    for ii in range(n_points):
        cloud.append(PointXYZ(0.0, 0.0, ii))


def everything_is_cloud(n_points: int):
    cloud = PointCloud(t.PointXYZ)
    values = np.hstack((
        np.zeros((n_points, 2)), np.arange(n_points).reshape(n_points, 1)
    ))
    cloud['position'] =  values


def compare_speed():
    """
    results:
    Run with 1 point
    Point-Based took 0.0 ms
    Cloud-Based took 0.2 ms
    Run with 10 point
    Point-Based took 0.0 ms
    Cloud-Based took 0.1 ms
    Run with 100 point
    Point-Based took 0.1 ms
    Cloud-Based took 0.0 ms
    Run with 1000 point
    Point-Based took 0.8 ms
    Cloud-Based took 0.1 ms
    Run with 10000 point
    Point-Based took 5.9 ms
    Cloud-Based took 0.5 ms
    Run with 100000 point
    Point-Based took 24.6 ms
    Cloud-Based took 2.9 ms
    Run with 1000000 point
    Point-Based took 194.7 ms
    Cloud-Based took 15.4 ms
    """
    for power in np.arange(7): 
        n_points = int(10**power)
        print(f"Run with {n_points} point")
        start = perf_counter()
        everything_is_point(n_points)
        stop = perf_counter()
        print(f"Point-Based took {(stop - start) * 1000:0.1f} ms")

        start = perf_counter()
        everything_is_cloud(n_points)
        stop = perf_counter()
        print(f"Cloud-Based took {(stop - start) * 1000:0.1f} ms")



if __name__ == "__main__":
    # test_general_cloud()
    compare_speed()
