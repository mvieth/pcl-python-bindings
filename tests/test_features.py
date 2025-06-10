# for pytest
from pcl.common import PointcloudXYZNormal
from pcl.features import NormalEstimationXYZNormal
from pcl.io import loadPCDFile
from math import isnan

def test_normalestimation():
    cloud_normals = PointcloudXYZNormal()
    loadPCDFile("/home/markus/pcd_files/table_scene_mug_stereo_textured.pcd", cloud_normals)
    ne = NormalEstimationXYZNormal()
    ne.setInputCloud(cloud_normals)
    ne.setKSearch(50)
    ne.compute(cloud_normals)
    for point in cloud_normals:
        if not isnan(point.x):
            assert abs(point.normal_x**2+point.normal_y**2+point.normal_z**2-1.0)<1e-5
