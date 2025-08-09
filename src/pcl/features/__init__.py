from .pcl_features_ext import (
	NormalEstimationXYZNormal,
	NormalEstimationXYZ,
	NormalEstimationXYZRGB,
	FPFHSignature33,
	PointcloudFPFHSignature33,
	FPFHEstimationXYZ,
	FPFHEstimationXYZRGB,
	FPFHEstimationXYZNormal,
)
from ..common import (
	PointCloud,
	PointXYZ, PointXYZNormal,
)

_PTYPE_TO_FPFH = {
	PointXYZ: (FPFHEstimationXYZ, False),
	PointXYZNormal: (FPFHEstimationXYZNormal, True),
}

class FPFH:
	"""Unified FPFH computation facade.

	Usage:
		fpfh = FPFH(k=20)               # or radius=0.05
		desc = fpfh(cloud, normals=normals_cloud_optional)

	If the input cloud has embedded normals (PointXYZNormal / PointXYZRGBNormal),
	external normals should not be supplied.
	"""
	__slots__ = ("_k","_radius")

	def __init__(self, *, k=None, radius=None):
		if (k is None) == (radius is None):
			raise ValueError("Exactly one of k or radius must be specified")
		self._k = k
		self._radius = radius

	def apply(self, cloud: PointCloud, normals: PointCloud | None = None):
		if cloud.point_type not in _PTYPE_TO_FPFH:
			raise TypeError(f"Unsupported point type for FPFH: {cloud.point_type.__name__}")
		est_cls, embedded = _PTYPE_TO_FPFH[cloud.point_type]
		if embedded and normals is not None:
			raise ValueError("Normals provided but input point type already has normals")
		if not embedded and normals is None:
			raise ValueError("Normals cloud required for this point type")
		est = est_cls()
		est.setInputCloud(cloud.native())
		if not embedded:
			est.setInputNormals(normals.native())
		if self._k is not None:
			est.setKSearch(int(self._k))
		else:
			est.setRadiusSearch(float(self._radius))
		out = PointcloudFPFHSignature33()
		est.compute(out)
		return out

	__call__ = apply

class NormalEstimation:
	"""Unified normal estimation facade.

	Automatically selects an underlying estimator based on the input cloud's point type:
	  PointXYZ -> NormalEstimationXYZ (output PointXYZNormal)
	  PointXYZRGB -> NormalEstimationXYZRGB (output PointXYZRGBNormal)
	  PointXYZNormal -> NormalEstimationXYZNormal (in-place type for output)
	  PointXYZRGBNormal -> NormalEstimationXYZRGBNormal (in-place type)

	Specify either k or radius (mutually exclusive).
	"""
	__slots__ = ("_k","_radius")

	def __init__(self, *, k=None, radius=None):
		if (k is None) == (radius is None):
			raise ValueError("Exactly one of k or radius must be specified")
		self._k = k; self._radius = radius

	def apply(self, cloud: PointCloud):
		pt = cloud.point_type
		if pt is PointXYZ:
			est_cls = NormalEstimationXYZ; out_type = PointXYZNormal
		elif pt is PointXYZNormal:
			est_cls = NormalEstimationXYZNormal; out_type = PointXYZNormal
		else:
			raise TypeError(f"Unsupported point type for normal estimation: {pt.__name__}")
		est = est_cls()
		est.setInputCloud(cloud.native())
		if self._k is not None:
			est.setKSearch(int(self._k))
		else:
			est.setRadiusSearch(float(self._radius))
		out = PointCloud(point_type=out_type)
		est.compute(out.native())
		return out

	__call__ = apply

__all__ = [
	'NormalEstimation',
	'NormalEstimationXYZ','NormalEstimationXYZRGB','NormalEstimationXYZNormal',
	'FPFHSignature33','PointcloudFPFHSignature33',
	'FPFHEstimationXYZ','FPFHEstimationXYZRGB','FPFHEstimationXYZNormal',
	'FPFH'
]
