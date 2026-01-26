"""Internal shared point type utilities (aliases & normalization).

This module centralizes point type alias handling so that both the
unified PointCloud facade and higher-level filters (e.g. PassThrough)
can resolve user-friendly string specifications without duplicating
mapping tables.
"""
# Point struct bindings live in subpackage 'common'; import them there.
from .pcl_common_ext import (
    PointXYZ, PointXYZI, PointXYZL,
    PointXYZRGBA, PointXYZRGB, PointXYZRGBL,
    PointXYZLAB, PointXYZHSV,
    Normal, PointXYZNormal, PointXYZRGBNormal, PointXYZINormal, PointXYZLNormal,
)

NAME_ALIASES = {
    'xyz': PointXYZ,
    'xyzi': PointXYZI,
    'xyzl': PointXYZL,
    'rgba': PointXYZRGBA,
    'rgb': PointXYZRGB,
    'xyzrgbl': PointXYZRGBL,
    'xyzlab': PointXYZLAB,
    'xyzhsv': PointXYZHSV,
    'normal': Normal,  # standalone normal
    'xyznormal': PointXYZNormal,
    'rgbnormal': PointXYZRGBNormal,
    'xyzinormal': PointXYZINormal,
    'xyzlnormal': PointXYZLNormal,
}

def normalize_point_type(spec, *, default=PointXYZ):
    """Return a concrete point type class from a user specification.

    spec may be:
      - None -> default
      - a point type class already
      - a string alias (case/underscore tolerant via NAME_ALIASES key rules)
    Raises ValueError / TypeError on unknown inputs.
    """
    if spec is None:
        return default
    if spec in NAME_ALIASES.values():  # already a class we know
        return spec  # type: ignore[return-value]
    if isinstance(spec, str):
        key = spec.lower().replace('_','')
        if key in NAME_ALIASES:
            return NAME_ALIASES[key]
        raise ValueError(f"Unknown point type alias: {spec}")
    raise TypeError("Invalid point_type specification")

__all__ = [
    'PointXYZ','PointXYZI','PointXYZL','PointXYZRGBA','PointXYZRGB','PointXYZRGBL',
    'PointXYZLAB','PointXYZHSV','Normal','PointXYZNormal','PointXYZRGBNormal','PointXYZINormal','PointXYZLNormal',
    'NAME_ALIASES','normalize_point_type'
]
