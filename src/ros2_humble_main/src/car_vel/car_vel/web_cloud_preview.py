"""Pure PointCloud2 display reduction; never writes to an algorithm topic."""
import copy
import numpy as np

CLOUD_TOPICS = ('/global_mapping_cloud', '/global_localization_map',
                '/cloud_mapping_dynamic', '/octomap_point_cloud_centers')
PROFILES = {'remote': (0.15, 20000, 3.0), 'lan': (0.05, 120000, 1.0)}


def thin_cloud(message, profile='remote', *, center=None, radius=15.0):
    voxel, maximum, _period = PROFILES[profile]
    width, height, step, row = message.width, message.height, message.point_step, message.row_step
    if width < 0 or height < 0 or not 1 <= step <= 256 or row < width * step:
        raise ValueError('invalid cloud layout')
    if len(message.data) < (max(0, height - 1) * row + width * step):
        raise ValueError('truncated cloud')
    # A point-count cap alone is insufficient when source records have many
    # extra fields. Bound the serialized raw body as well (before base64).
    maximum = min(maximum, (512 * 1024 if profile == 'remote' else 4 * 1024 * 1024) // step)
    if not width or not height:
        result = copy.copy(message)
        result.height, result.width, result.row_step, result.data = 1, 0, 0, b''
        return result
    fields = {field.name: field for field in message.fields}
    xyz = []
    for name in ('x', 'y', 'z'):
        field = fields.get(name)
        if field is None or field.datatype not in (7, 8) or field.count != 1:
            raise ValueError('cloud requires scalar floating XYZ')
        dtype = np.dtype(('>' if message.is_bigendian else '<')
                         + ('f4' if field.datatype == 7 else 'f8'))
        if field.offset < 0 or field.offset + dtype.itemsize > step:
            raise ValueError('invalid field offset')
        xyz.append(np.ndarray((height, width), dtype=dtype, buffer=message.data,
                              offset=field.offset, strides=(row, step)).reshape(-1))
    points = np.column_stack(xyz)
    valid = np.isfinite(points).all(axis=1) & (np.abs(points) < 1e8).all(axis=1)
    if center is not None:
        valid &= np.linalg.norm(points[:, :2] - np.asarray(center[:2]), axis=1) <= radius
    indexes = np.flatnonzero(valid)
    if indexes.size:
        cells = np.floor(points[indexes] / voxel).astype(np.int64)
        _, selected = np.unique(cells, axis=0, return_index=True)
        indexes = indexes[np.sort(selected)]
        if indexes.size > maximum:
            indexes = indexes[np.linspace(0, indexes.size - 1, maximum, dtype=np.int64)]
    records = np.ndarray((height, width), dtype=np.dtype(('V', step)),
                         buffer=message.data, strides=(row, step)).reshape(-1)
    result = copy.copy(message)
    result.height, result.width = 1, int(indexes.size)
    result.row_step = result.width * step
    result.data = records[indexes].tobytes()
    result.is_dense = True
    return result
