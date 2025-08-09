from ._pointtypes import (
	PointXYZ, PointXYZI, PointXYZL, PointXYZRGBA, PointXYZRGB, PointXYZRGBL,
	PointXYZLAB, PointXYZHSV, Normal, PointXYZNormal, PointXYZRGBNormal, PointXYZINormal, PointXYZLNormal,
	NAME_ALIASES, normalize_point_type,
)
# Cloud container bindings (PointcloudXYZ*, etc.)
from .pcl_common_ext import (
	PointcloudXYZ, PointcloudXYZI, PointcloudXYZL, PointcloudXYZRGBA, PointcloudXYZRGB, PointcloudXYZRGBL,
	PointcloudXYZLAB, PointcloudXYZHSV, PointcloudXYZNormal, PointcloudXYZRGBNormal, PointcloudXYZINormal, PointcloudXYZLNormal,
)

_POINT_CLASS_TO_CLOUD = {
	PointXYZ: PointcloudXYZ,
	PointXYZI: PointcloudXYZI,
	PointXYZL: PointcloudXYZL,
	PointXYZRGBA: PointcloudXYZRGBA,
	PointXYZRGB: PointcloudXYZRGB,
	PointXYZRGBL: PointcloudXYZRGBL,
	PointXYZLAB: PointcloudXYZLAB,
	PointXYZHSV: PointcloudXYZHSV,
	PointXYZNormal: PointcloudXYZNormal,
	PointXYZRGBNormal: PointcloudXYZRGBNormal,
	PointXYZINormal: PointcloudXYZINormal,
	PointXYZLNormal: PointcloudXYZLNormal,
}

def _deduce_point_type_from_sample(p):
	for pt in _POINT_CLASS_TO_CLOUD:
		if isinstance(p, pt):
			return pt
	raise TypeError("Unsupported point instance for automatic type deduction")

def _resolve_point_type(spec):
	if spec in _POINT_CLASS_TO_CLOUD:
		return spec
	pt = normalize_point_type(spec)
	if pt not in _POINT_CLASS_TO_CLOUD:
		raise TypeError("Point type not bound in this build")
	return pt

class PointCloud:
	"""Unified point cloud.

	Construction forms:
		PointCloud() -> empty XYZ cloud
		PointCloud(point_type=PointXYZRGB)
		PointCloud(points=[PointXYZ(...), ...])  # auto-deduce type from first element
		PointCloud(points=iterable, point_type='rgb')  # string alias
	"""
	__slots__ = ("_cloud", "_point_type")

	def __init__(self, points=None, *, point_type=None, width=None, height=None, is_dense=None):
		if points is not None and point_type is None:
			# Attempt deduction from first element (peek)
			it = iter(points)
			try:
				first = next(it)
			except StopIteration:
				point_type = PointXYZ
				points = []
			else:
				pt = _deduce_point_type_from_sample(first)
				point_type = pt
				# Rebuild iterator with first element
				points = [first, *it]
		point_type = _resolve_point_type(point_type)
		self._point_type = point_type
		cloud_cls = _POINT_CLASS_TO_CLOUD[point_type]
		self._cloud = cloud_cls()
		if width is not None: self._cloud.width = int(width)
		if height is not None: self._cloud.height = int(height)
		if is_dense is not None: self._cloud.is_dense = bool(is_dense)
		if points:
			self.extend(points)

	# Mutating ops
	def append(self, p):
		# lenient: if a different point struct instance is provided but has same layout, rely on C++ side safety
		if not isinstance(p, self._point_type):
			raise TypeError(f"Expected point of type {self._point_type.__name__}")
		self._cloud.append(p)

	def extend(self, iterable):
		for p in iterable:
			self.append(p)

	def reserve(self, n):
		self._cloud.reserve(n)

	def resize(self, n):
		self._cloud.resize(n)

	def clear(self):
		self._cloud.clear()

	# Access
	def __len__(self): return len(self._cloud)
	@property
	def size(self): return self._cloud.size
	def __getitem__(self, idx): return self._cloud[idx]
	def __iter__(self): return iter(self._cloud)

	# Meta
	@property
	def point_type(self): return self._point_type
	@property
	def width(self): return self._cloud.width
	@width.setter
	def width(self, v): self._cloud.width = int(v)
	@property
	def height(self): return self._cloud.height
	@height.setter
	def height(self, v): self._cloud.height = int(v)
	@property
	def is_dense(self): return self._cloud.is_dense
	@is_dense.setter
	def is_dense(self, v): self._cloud.is_dense = bool(v)
	def native(self): return self._cloud

	def __repr__(self):
		return f"PointCloud(size={len(self)}, type={self._point_type.__name__}, width={self.width}, height={self.height}, is_dense={self.is_dense})"

def load_points(points, point_type=None):
	return PointCloud(points=points, point_type=point_type)

__all__ = [
	'PointCloud', 'load_points',
	# backward compatible specialized cloud classes
	'PointcloudXYZ','PointcloudXYZI','PointcloudXYZL','PointcloudXYZRGBA','PointcloudXYZRGB','PointcloudXYZRGBL',
	'PointcloudXYZLAB','PointcloudXYZHSV','PointcloudXYZNormal','PointcloudXYZRGBNormal','PointcloudXYZINormal','PointcloudXYZLNormal',
	# point structs
	'PointXYZ','PointXYZI','PointXYZL','PointXYZRGBA','PointXYZRGB','PointXYZRGBL','PointXYZLAB','PointXYZHSV','Normal',
	'PointXYZNormal','PointXYZRGBNormal','PointXYZINormal','PointXYZLNormal'
]

