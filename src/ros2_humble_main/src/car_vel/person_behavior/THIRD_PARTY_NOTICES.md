# Runtime and model provenance

This directory adapts the supplied local `person-behavior-recognition` project
for the robot's isolated **CPU fall/lying, smoking and scene-fight** workers.
The source project was read only. Original file hashes and adaptation scope are
recorded in `environment/source_provenance.json`.

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
- **Ultralytics YOLOv8m-pose**: official weight exported by the supplied project
  with Ultralytics `8.3.228`, static FP32 ONNX, 640 pixels, opset 17, no NMS.
  Its AGPL-3.0 license is preserved in `licenses/Ultralytics-AGPL-3.0.txt` from
  https://github.com/ultralytics/ultralytics/blob/v8.3.228/LICENSE. The source
  model and export provenance are in `models_manifest.json`; the `.pt` export
  source is not included or loaded. ROS package metadata does not relicense
  this model or its upstream code.
- **PaddleVideo ppTSM**: https://github.com/PaddlePaddle/PaddleVideo. The supplied
  scene-fighting model is the official `PaddleVideo-release2.3/ppTSM_fight.zip`
  export recorded in `models_manifest.json`.

The supplied v4 PPYOLOE smoking export and pure mouth/hand association modules
were added on 2026-11-02. Provenance and copied-module hashes are in
`environment/smoking_source_provenance.json`; weight hashes/URL are in the model
and asset manifests. Shared pose, PPYOLOE smoking and ppTSM fight are loaded only
when their branches are enabled. No legacy MOT, HRNet or STGCN model is loaded.
The archive contains upstream general-purpose code which may name other
capabilities; that does not enable those capabilities in this runtime.

Bundled asset SHA256 values were checked both before and after copying, and
are checked again by the explicit installer. These are local integrity checks,
not publisher signatures. Dependencies and models retain their own licenses;
the included Apache text does not license all bundled material under Apache.
