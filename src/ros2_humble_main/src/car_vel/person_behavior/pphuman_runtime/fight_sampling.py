"""Eight real capture frames, scaled like the pinned PP-TSM frame-list path."""
from collections import deque
import math

import numpy as np
from PIL import Image


def scale_capture_frame(bgr, short):
    height, width = bgr.shape[:2]
    long = int(short * 4 / 3)
    size = (short, long) if width <= height else (long, short)
    rgb = Image.fromarray(bgr[:, :, ::-1]).resize(size, getattr(Image, "Resampling", Image).BILINEAR)
    return np.asarray(rgb).copy()


class FightSampler:
    def __init__(self, cfg):
        self.short = cfg['short_size']
        hz = cfg['sample_hz']
        if self.short not in (238, 340) or cfg['window_frames'] != 8 or not math.isfinite(hz) or not 0 < hz <= 30:
            raise ValueError('Capture sampling requires 8 frames, short side 238/340 and bounded rate')
        self.period = round(1e9 / hz)
        self.gap = round(cfg['max_sample_gap_seconds'] * 1e9)
        self.max_span = round(cfg['max_window_seconds'] * 1e9)
        self.samples = deque(maxlen=8)
        self.clear()

    def clear(self):
        self.samples.clear()
        self.next_ns = self.last_ns = None

    def observe(self, bgr, stamp):
        if self.last_ns is not None and (stamp <= self.last_ns or stamp-self.last_ns > self.gap):
            self.clear()
        self.last_ns = stamp
        if self.next_ns is None or stamp >= self.next_ns:
            self.samples.append((stamp, scale_capture_frame(bgr, self.short)))
            self.next_ns = stamp+self.period if self.next_ns is None else self.next_ns+((stamp-self.next_ns)//self.period+1)*self.period
        while self.samples and stamp - self.samples[0][0] > self.max_span:
            self.samples.popleft()
        return tuple(self.samples)


def validate_samples(samples, stamp, cfg):
    if not isinstance(samples, (list, tuple)) or len(samples)>8:
        raise ValueError('At most eight capture samples are allowed')
    previous = -1
    short = cfg['short_size']; long = int(short*4/3)
    for source_stamp, image in samples:
        if (type(source_stamp) is not int or not previous < source_stamp <= stamp or
                stamp-source_stamp > cfg['max_window_seconds']*1e9 or
                not isinstance(image,np.ndarray) or image.dtype!=np.uint8 or
                image.shape not in ((short,long,3),(long,short,3))):
            raise ValueError('Invalid capture frame, order, age or scaled RGB shape')
        previous = source_stamp
