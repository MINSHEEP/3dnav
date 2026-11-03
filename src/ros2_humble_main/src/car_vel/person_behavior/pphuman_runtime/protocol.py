"""Version 1 length-prefixed JSON + BGR protocol, independent of Paddle."""

import json
import struct


class ProtocolError(ValueError):
    pass


def read_exact(stream, length, allow_eof=False):
    chunks = bytearray()
    while len(chunks) < length:
        chunk = stream.read(length - len(chunks))
        if not chunk:
            if allow_eof and not chunks:
                return None
            raise ProtocolError("Truncated protocol packet")
        chunks.extend(chunk)
    return bytes(chunks)


def read_request(stream, max_header_bytes=65536, max_payload_bytes=25165824,
                 max_dimension=4096):
    prefix = read_exact(stream, 4, allow_eof=True)
    if prefix is None:
        return None
    size, = struct.unpack(">I", prefix)
    if size < 2 or size > max_header_bytes:
        raise ProtocolError("JSON header length exceeds permitted bounds")
    try:
        header = json.loads(read_exact(stream, size).decode("utf-8"))
    except (ValueError, UnicodeError) as exc:
        raise ProtocolError("Invalid JSON header") from exc
    if not isinstance(header, dict) or header.get("op") not in {"infer", "reset", "shutdown"}:
        raise ProtocolError("Expected infer, reset or shutdown object")
    length = header.get("payload_bytes", 0)
    if type(length) is not int or not 0 <= length <= max_payload_bytes:
        raise ProtocolError("Invalid payload_bytes")
    if header["op"] != "infer":
        if length:
            raise ProtocolError("Control requests cannot contain image bytes")
        return header, b""
    for key in ("request_id", "source_epoch"):
        if key not in header or type(header[key]) not in (int, str):
            raise ProtocolError("Missing/invalid " + key)
    if type(header.get("stamp_ns")) is not int or header["stamp_ns"] < 0:
        raise ProtocolError("stamp_ns must be a nonnegative integer")
    branches = header.get("enabled_branches")
    if branches is not None and (not isinstance(branches, dict)
            or not set(branches).issubset({"fall", "fight", "smoking"})
            or any(type(value) is not bool for value in branches.values())):
        raise ProtocolError("enabled_branches accepts only boolean fall/fight/smoking switches")
    shape = header.get("shape")
    if (not isinstance(shape, list) or len(shape) != 3
            or any(type(v) is not int for v in shape)
            or shape[2] != 3 or not all(0 < v <= max_dimension for v in shape[:2])):
        raise ProtocolError("Expected bounded H,W,3 shape")
    stamps = header.get("clip_stamps_ns")
    count = 1
    if stamps is not None:
        if (not isinstance(stamps, list) or not 1 <= len(stamps) <= 8
                or any(type(v) is not int or v < 0 for v in stamps)
                or any(a >= b for a, b in zip(stamps, stamps[1:]))
                or stamps[-1] != header["stamp_ns"]
                or not isinstance(branches, dict) or branches.get("fall", False)
                or branches.get("smoking", False) or branches.get("fight") is not True):
            raise ProtocolError("Expected bounded, ordered fight-only capture clip")
        count = len(stamps)
    expected = count * shape[0] * shape[1] * 3
    if "fight_samples" in header:
        samples = header["fight_samples"]
        if stamps is not None or not isinstance(samples, list) or len(samples) > 8:
            raise ProtocolError("Expected at most eight RGB capture samples and no legacy clip")
        previous = -1
        for sample in samples:
            if not isinstance(sample, dict):
                raise ProtocolError("Invalid fight sample header")
            stamp, dims = sample.get("stamp_ns"), sample.get("shape")
            if (type(stamp) is not int or not previous < stamp <= header["stamp_ns"]
                    or not isinstance(dims, list) or len(dims) != 3
                    or any(type(value) is not int for value in dims) or dims[2] != 3
                    or not all(0 < value <= 512 for value in dims[:2])):
                raise ProtocolError("Invalid fight sample timestamp or shape")
            expected += dims[0] * dims[1] * 3
            previous = stamp
    if length != expected:
        raise ProtocolError("payload_bytes does not match the declared frame shapes")
    return header, read_exact(stream, length)


def write_message(stream, message):
    encoded = json.dumps(message, ensure_ascii=False, allow_nan=False,
                         separators=(",", ":")).encode("utf-8")
    stream.write(struct.pack(">I", len(encoded)))
    stream.write(encoded)
    stream.flush()


def read_message(stream, max_bytes=16 * 1024 * 1024):
    """Useful to integration clients and protocol tests."""
    prefix = read_exact(stream, 4, allow_eof=True)
    if prefix is None:
        return None
    length, = struct.unpack(">I", prefix)
    if not 0 < length <= max_bytes:
        raise ProtocolError("Invalid result length")
    return json.loads(read_exact(stream, length).decode("utf-8"))
