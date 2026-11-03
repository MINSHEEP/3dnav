# Runtime and model provenance

This directory adapts the supplied local `person-behavior-recognition` project
for the robot's isolated **CPU fall/lying, smoking and scene-fight** workers.
The source project was read only. Original file hashes and adaptation scope are
recorded in `assets/runtime_assets.json` and the per-model export, conversion and quantization JSON files. Earlier integration provenance is retained under `environment/`.

- **PaddleDetection / PP-Human**: https://github.com/PaddlePaddle/PaddleDetection,
  commit `b25522a0f4bde8c80603f3ba5e3472059972e3b5`. The supplied pinned archive is
  bundled unchanged. `licenses/Apache-2.0.txt` retains its license. The installer
  extracts regular files and skips README links; it never runs upstream setup
  scripts. `environment/upstream_source.json` retains the supplied provenance.
- **zhahoi/yolov8-pose-fall-detection**:
  https://github.com/zhahoi/yolov8-pose-fall-detection. Geometric fall rules were
  adapted by the supplied project, including observability checks, temporal
  confirmation, tracking and projected lying posture. Its upstream license is
  Apache-2.0: https://github.com/zhahoi/yolov8-pose-fall-detection/blob/main/LICENSE.
- **Ultralytics YOLO26s-pose**: the supplied official weights were exported to
  static OpenVINO FP32 at 640 pixels. Export and source-weight details are in
  `output_inference/yolo26s_pose_openvino_fp32/export.json`. The retained
  `licenses/Ultralytics-AGPL-3.0.txt` applies to the Ultralytics model.
- **PaddleVideo ppTSM**: the supplied OpenVINO conversion uses mixed INT8 with
  an FP32 input/output contract. Details are in
  `output_inference/ppTSM_int8_224_cpu_candidate/quantization.json`.
- **PP-YOLOE smoking**: the supplied FP32 OpenVINO conversion uses the original
  PP-YOLOE weights and the current mouth/hand and profile association rules.
  See `output_inference/ppyoloe_smoking_openvino_fp32/conversion.json`.

The archive contains upstream general-purpose code which may name other
capabilities; that does not enable those capabilities in this runtime.

Bundled asset SHA256 values were checked both before and after copying, and
are checked again by the explicit installer. These are local integrity checks,
not publisher signatures. Dependencies and models retain their own licenses;
the included Apache text does not license all bundled material under Apache.
