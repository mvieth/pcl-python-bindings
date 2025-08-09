from .pcl_filters_ext import (
	PassThroughXYZ, PassThroughXYZRGBA, PassThroughXYZRGB, PassThroughXYZNormal, PassThroughXYZRGBNormal,
)
from ..common._pointtypes import (
	PointXYZ, PointXYZRGBA, PointXYZRGB, PointXYZNormal, PointXYZRGBNormal,
	normalize_point_type,
)
from ..common import PointCloud

_PTYPE_TO_IMPL = {
	PointXYZ: PassThroughXYZ,
	PointXYZRGBA: PassThroughXYZRGBA,
	PointXYZRGB: PassThroughXYZRGB,
	PointXYZNormal: PassThroughXYZNormal,
	PointXYZRGBNormal: PassThroughXYZRGBNormal,
}

def _resolve_point_type(spec, sample_cloud=None):
	if spec is None and sample_cloud is not None:
		return sample_cloud.point_type
	if spec in _PTYPE_TO_IMPL:
		return spec
	pt = normalize_point_type(spec)
	if pt not in _PTYPE_TO_IMPL:
		raise TypeError("Point type not supported by PassThrough in this build")
	return pt

class PassThrough:
	"""Unified PassThrough filter with lazy point type deduction.

	Typical usage:
		cloud = PointCloud(points=[PointXYZ(0,0,0), PointXYZ(0,0,1)])
		f = PassThrough(field='z', limits=(0.2, 0.8))  # no point_type needed
		f.setInputCloud(cloud)  # binds to cloud's point type automatically
		result = f()            # or f.apply() -> returns new filtered PointCloud

	Chaining:
		result2 = PassThrough(field='z', limits=(0.0, 0.5)).setInputCloud(result)()

	You may still force a point type ahead of time via PassThrough(point_type='rgb'),
	but it's optional; calling setInputCloud first will set it when unset.
	"""
	__slots__ = ('_impl','_point_type','_field','_limits','_negative','_cloud')

	def __init__(self, *, point_type=None, field='z', limits=(0.0, 1.0), negative=False):
		self._point_type = None
		self._impl = None
		self._cloud = None
		self._field = field
		self._limits = limits
		self._negative = bool(negative)
		if point_type is not None:
			self._initialize_impl(_resolve_point_type(point_type))

	def _initialize_impl(self, pt):
		if self._impl is not None:
			if pt is not self._point_type:
				raise RuntimeError('Filter already initialized with different point type')
			return
		self._point_type = pt
		self._impl = _PTYPE_TO_IMPL[pt]()
		self._impl.setFilterFieldName(self._field)
		self._impl.setFilterLimits(float(self._limits[0]), float(self._limits[1]))
		self._impl.setNegative(self._negative)

	def setInputCloud(self, cloud: PointCloud):
		self._initialize_impl(cloud.point_type)
		self._cloud = cloud
		return self  # allow chaining

	def set_field(self, name):
		self._field = name
		if self._impl: self._impl.setFilterFieldName(name)
		return self

	def set_limits(self, min_v, max_v):
		self._limits = (min_v, max_v)
		if self._impl: self._impl.setFilterLimits(float(min_v), float(max_v))
		return self

	def set_negative(self, flag):
		self._negative = bool(flag)
		if self._impl: self._impl.setNegative(self._negative)
		return self

	def apply(self, output=None):
		if self._cloud is None:
			raise RuntimeError('Input cloud not set. Call setInputCloud first.')
		# impl guaranteed initialized by setInputCloud
		if output is None:
			output = PointCloud(point_type=self._point_type)
		self._impl.setInputCloud(self._cloud.native())
		self._impl.filter(output.native())
		return output

	def __call__(self, *args, **kwargs):
		return self.apply(*args, **kwargs)

__all__ = [
	'PassThrough',
	'PassThroughXYZ','PassThroughXYZRGBA','PassThroughXYZRGB','PassThroughXYZNormal','PassThroughXYZRGBNormal'
]
