#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "错误：随程序提供的 HCNetSDK 仅支持 x86_64。" >&2
  exit 1
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
DEPENDENCY_DIR="${HIKVISION_PYTHONPATH:-${HOME}/.local/share/hikvision-camera/python}"
PIP_TIMEOUT_SECONDS="${HIKVISION_PIP_TIMEOUT:-600}"
PIP_RETRY_COUNT="${HIKVISION_PIP_RETRIES:-10}"
PYPI_INDEX_URL="${HIKVISION_PYPI_INDEX_URL:-${PIP_INDEX_URL:-https://pypi.org/simple}}"
PYTORCH_CPU_INDEX_URL="${HIKVISION_PYTORCH_CPU_INDEX_URL:-https://download.pytorch.org/whl/cpu}"
CPU_CONSTRAINTS_FILE="${PACKAGE_DIR}/requirements-pytorch-cpu.txt"
SFACE_MODEL_PATH="${HIKVISION_SFACE_MODEL_PATH:-${PACKAGE_DIR}/hikvision_camera/models/face_recognition_sface_2021dec.onnx}"
SFACE_MODEL_URL="https://media.githubusercontent.com/media/opencv/opencv_zoo/main/models/face_recognition_sface/face_recognition_sface_2021dec.onnx"
SFACE_MODEL_SHA256="0ba9fbfa01b5270c96627c4ef784da859931e02f04419c829e83484087c34e79"

sudo apt-get update
sudo apt-get install -y \
  fastdds-tools \
  python3-pip \
  python3-tk \
  curl \
  ffmpeg \
  libuuid1 \
  libgl1 \
  libx11-6 \
  libbz2-1.0 \
  zlib1g \
  libfreetype6 \
  libstdc++6 \
  libgcc-s1 \
  libasound2

# 继续使用 Ubuntu 系统 Python，但把相机依赖放进独立目录。这不是虚拟环境，
# 同时可以避免新版 NumPy/OpenCV 覆盖 ROS2 Humble 的 Python 依赖。
mkdir -p "${DEPENDENCY_DIR}"
python3 -m pip install --upgrade --target "${DEPENDENCY_DIR}" \
  --timeout "${PIP_TIMEOUT_SECONDS}" \
  --retries "${PIP_RETRY_COUNT}" \
  --index-url "${PYPI_INDEX_URL}" \
  filelock \
  'typing-extensions>=4.10.0' \
  'sympy>=1.13.3' \
  'networkx>=2.5.1' \
  jinja2 \
  'fsspec>=0.8.5'
python3 -m pip install --upgrade --target "${DEPENDENCY_DIR}" \
  --timeout "${PIP_TIMEOUT_SECONDS}" \
  --retries "${PIP_RETRY_COUNT}" \
  --index-url "${PYTORCH_CPU_INDEX_URL}" \
  --no-deps \
  torch==2.10.0 torchvision==0.25.0
PYTHONPATH="${DEPENDENCY_DIR}${PYTHONPATH:+:${PYTHONPATH}}" \
python3 -m pip install --upgrade --target "${DEPENDENCY_DIR}" \
  --timeout "${PIP_TIMEOUT_SECONDS}" \
  --retries "${PIP_RETRY_COUNT}" \
  --index-url "${PYPI_INDEX_URL}" \
  --extra-index-url "${PYTORCH_CPU_INDEX_URL}" \
  --constraint "${CPU_CONSTRAINTS_FILE}" \
  -r "${PACKAGE_DIR}/requirements-hikvision-camera.txt"

mkdir -p "$(dirname -- "${SFACE_MODEL_PATH}")"
SFACE_ACTUAL_SHA256=""
if [[ -f "${SFACE_MODEL_PATH}" ]]; then
  SFACE_ACTUAL_SHA256="$(sha256sum "${SFACE_MODEL_PATH}" | awk '{print $1}')"
fi
if [[ "${SFACE_ACTUAL_SHA256}" != "${SFACE_MODEL_SHA256}" ]]; then
  SFACE_TEMP_PATH="$(mktemp "${SFACE_MODEL_PATH}.download.XXXXXX")"
  trap 'rm -f -- "${SFACE_TEMP_PATH:-}"' EXIT
  curl --fail --location --retry 5 --retry-all-errors \
    --output "${SFACE_TEMP_PATH}" "${SFACE_MODEL_URL}"
  echo "${SFACE_MODEL_SHA256}  ${SFACE_TEMP_PATH}" | sha256sum --check --status
  install -m 0644 "${SFACE_TEMP_PATH}" "${SFACE_MODEL_PATH}"
  rm -f -- "${SFACE_TEMP_PATH}"
  trap - EXIT
fi

SFACE_MODEL_PATH="${SFACE_MODEL_PATH}" \
PYTHONPATH="${DEPENDENCY_DIR}${PYTHONPATH:+:${PYTHONPATH}}" python3 - <<'PY'
import cv2
import hyperlpr3
import onnxruntime
import os
import torch
import ultralytics

if torch.version.cuda is not None:
    raise SystemExit(
        f"错误：安装到了 CUDA 版 PyTorch ({torch.__version__})，请改用 CPU 版依赖"
    )

sface_path = os.environ["SFACE_MODEL_PATH"]
recognizer = cv2.FaceRecognizerSF.create(sface_path, "")
if recognizer is None:
    raise SystemExit("错误：OpenCV未能加载SFace模型")
catcher = hyperlpr3.LicensePlateCatcher()
if catcher is None:
    raise SystemExit("错误：HyperLPR3初始化失败")

print("相机 Python 依赖安装完成")
print(f"OpenCV: {cv2.__version__}")
print(f"ONNX Runtime: {onnxruntime.__version__}")
print(f"SFace: {sface_path}")
print("HyperLPR3: LicensePlateCatcher 已就绪")
print(f"PyTorch: {torch.__version__}; CUDA 可用: {torch.cuda.is_available()}")
print(f"Ultralytics: {ultralytics.__version__}")
PY
