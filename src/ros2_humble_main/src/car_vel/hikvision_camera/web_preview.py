"""Browser-only JPEG previews. No camera ownership or inference settings here."""
from dataclasses import dataclass
import threading
import time


@dataclass(frozen=True)
class Profile:
    width: int
    height: int
    fps: int
    quality: int


PROFILES = {'remote': Profile(640, 360, 5, 55),
            'lan': Profile(1280, 720, 15, 82)}


def encode_preview(jpeg, profile):
    # The camera runtime already requires cv2/numpy. Import lazily so this
    # module and its scheduling contract can be tested without the SDK stack.
    import cv2
    import numpy as np
    frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError('预览源不是有效JPEG')
    height, width = frame.shape[:2]
    ratio = min(1.0, profile.width / width, profile.height / height)
    if ratio < 1.0:
        frame = cv2.resize(frame, (max(1, int(width * ratio)), max(1, int(height * ratio))),
                           interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, profile.quality])
    if not ok:
        raise RuntimeError('网页预览编码失败')
    return encoded.tobytes()


class PreviewCache:
    """Six bounded shared entries, computed only when a browser asks for them.

    Locks are per source/profile; no network write happens under a cache lock.
    Slow clients cannot queue old frames or block a different preview. Internal
    consumers continue using the original endpoints, without touching this cache.
    """
    def __init__(self, readers, *, encoder=encode_preview, clock=time.monotonic):
        self.readers, self.encoder, self.clock = readers, encoder, clock
        self.entries = {(source, name): {'lock': threading.Lock(), 'at': None,
                        'jpeg': None, 'sequence': 0}
                        for source in readers for name in PROFILES}

    def frame(self, source, profile_name):
        if (source, profile_name) not in self.entries:
            raise ValueError('未知预览来源或档位')
        profile = PROFILES[profile_name]
        entry = self.entries[source, profile_name]
        with entry['lock']:
            now = self.clock()
            if entry['at'] is None or now - entry['at'] >= 1.0 / profile.fps:
                original = self.readers[source]()
                if not original:
                    raise RuntimeError('预览源尚无有效帧')
                image = self.encoder(original, profile)
                # Failure must never turn a stale frame into a fresh one.
                entry.update(jpeg=image, at=self.clock(), sequence=entry['sequence'] + 1)
            return entry['sequence'], entry['jpeg']
