#!/usr/bin/env python3
"""Launch with cyber1_vc's virtual-environment Python and --voice-project."""

from pathlib import Path
import sys


# Supports the source tree as well as an installed ROS package.
source = Path(__file__).resolve().parent.parent
if (source / "car_vel" / "voice_debug_server.py").is_file():
    sys.path.insert(0, str(source))

from car_vel.voice_debug_server import main


if __name__ == "__main__":
    raise SystemExit(main())
