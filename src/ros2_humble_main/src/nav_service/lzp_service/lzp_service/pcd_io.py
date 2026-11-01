"""Bounded NumPy reader for the PCL PCD files used by localization maps."""

from __future__ import annotations

from pathlib import Path

import numpy as np


_TYPE_CODES = {
    ('F', 4): '<f4', ('F', 8): '<f8',
    ('I', 1): '<i1', ('I', 2): '<i2', ('I', 4): '<i4', ('I', 8): '<i8',
    ('U', 1): '<u1', ('U', 2): '<u2', ('U', 4): '<u4', ('U', 8): '<u8',
}


def _parse_header(stream):
    values = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('PCD header ended before DATA')
        if len(line) > 16_384:
            raise ValueError('PCD header line is too long')
        try:
            text = line.decode('ascii').strip()
        except UnicodeDecodeError as error:
            raise ValueError('PCD header must be ASCII') from error
        if not text or text.startswith('#'):
            continue
        parts = text.split()
        key = parts[0].upper()
        values[key] = parts[1:]
        if key == 'DATA':
            return values


def load_pcd_xyz(path, *, maximum_points=1_000_000, maximum_bytes=64 * 1024 * 1024):
    """Load finite x/y/z points from one bounded ASCII or binary PCD file."""

    source = Path(path)
    stat = source.stat()
    if stat.st_size <= 0 or stat.st_size > int(maximum_bytes):
        raise ValueError(
            f'PCD size {stat.st_size} is outside the permitted range')
    with source.open('rb') as stream:
        header = _parse_header(stream)
        fields = header.get('FIELDS') or header.get('FIELD') or []
        sizes = [int(value) for value in header.get('SIZE', [])]
        types = [value.upper() for value in header.get('TYPE', [])]
        counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
        if not fields or not (
                len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError('PCD field metadata is inconsistent')
        if any(count != 1 for count in counts):
            raise ValueError('PCD fields with COUNT other than 1 are unsupported')
        try:
            point_count = int((header.get('POINTS') or [''])[0])
        except (TypeError, ValueError, IndexError) as error:
            raise ValueError('PCD POINTS is invalid') from error
        if point_count <= 0 or point_count > int(maximum_points):
            raise ValueError(
                f'PCD point count {point_count} is outside the permitted range')
        if not {'x', 'y', 'z'}.issubset(fields):
            raise ValueError('PCD must contain x/y/z fields')
        formats = []
        for field, size, type_code in zip(fields, sizes, types):
            dtype = _TYPE_CODES.get((type_code, size))
            if dtype is None:
                raise ValueError(
                    f'unsupported PCD field type: {field} {type_code}{size}')
            formats.append(dtype)
        structured = np.dtype({'names': fields, 'formats': formats})
        data_mode = (header.get('DATA') or [''])[0].lower()
        if data_mode == 'binary':
            expected = point_count * structured.itemsize
            payload = stream.read(expected + 1)
            if len(payload) != expected:
                raise ValueError(
                    f'PCD binary payload has {len(payload)} bytes, expected {expected}')
            records = np.frombuffer(payload, dtype=structured, count=point_count)
            points = np.column_stack((records['x'], records['y'], records['z']))
        elif data_mode == 'ascii':
            columns = np.loadtxt(stream, dtype=np.float64, max_rows=point_count + 1)
            columns = np.atleast_2d(columns)
            if columns.shape != (point_count, len(fields)):
                raise ValueError('PCD ASCII row or column count is inconsistent')
            indices = [fields.index(name) for name in ('x', 'y', 'z')]
            points = columns[:, indices]
        elif data_mode == 'binary_compressed':
            raise ValueError('binary_compressed PCD is not supported')
        else:
            raise ValueError(f'unsupported PCD DATA mode: {data_mode}')
    points = np.asarray(points, dtype=np.float32)
    finite = points[np.isfinite(points).all(axis=1)]
    if finite.shape[0] < 3:
        raise ValueError('PCD does not contain enough finite x/y/z points')
    return finite

