"""Serialize localization display clouds without invented intensity or RGB fields."""

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField


def make_point_cloud(header, points):
    points = np.asarray(points)
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError("Localization cloud must have XYZ or XYZ-intensity columns")
    fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z"))
    ]
    if points.shape[1] == 4:
        fields.append(
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1)
        )
    data = np.ascontiguousarray(points, dtype="<f4")
    point_step = len(fields) * 4
    return PointCloud2(
        header=header,
        height=1,
        width=len(data),
        fields=fields,
        is_bigendian=False,
        point_step=point_step,
        row_step=point_step * len(data),
        data=data.tobytes(),
        is_dense=bool(np.isfinite(data).all()),
    )
