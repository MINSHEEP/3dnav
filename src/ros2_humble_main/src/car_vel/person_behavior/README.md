# CPU fall/lying, smoking and scene fight runtime

The gateway owns the camera and feeds two independent CPU processes. The pose
worker shares one YOLOv8m-pose ONNX session between fall/lying rules and v4 smoking
association, with a CPU PPYOLOE cigarette predictor. The independent fight worker
loads ppTSM only. Neither opens a camera or ROS node; each defaults to two threads.
Use `--branch pose` and `--branch fight`, with `--branches` selecting enabled
models within that scope. Disabled predictors are not constructed. Switch changes
restart only the affected owned worker. Existing two-switch settings migrate to
smoking enabled. UI/inspection speech, reports and evidence use category `smoking`.

Smoking uses the supplied v4 mouth/hand/near-face evidence with real source
timestamps. Its score is an associated cigarette detector score, never an action
probability. Short-occlusion/cached results may be shown but never issue new alarms.
The old primary YOLO cigarette class is suppressed; shared fire/smoke weights stay.

The gateway samples eight REAL frames on the capture clock for fighting, rather
than sampling only frames surviving slow pose inference. A bounded ring retains
at most eight aspect-preserving BGR images with longest side at most 640 pixels.
Inference receives a snapshot of that ring. The original 0.6-second maximum
sample gap, three-second clip span and one-second fight-result TTL still apply;
no frame padding or timestamp synthesis is used. A camera/source/config/task
change clears pending captures and evidence. Pose rules still require their
original observation cadence; this change does not cure insufficient pose FPS.

`/api/behavior/status` now reports independent `workers`, per-branch freshness,
capture/window counts, model timings, expired-result counts and `preview_branch`.
The preview always belongs to ONE worker's exact inference frame. Scene results
are never drawn on a different worker's pose frame. Inspection saves each
branch's original/annotated evidence pair separately. Model failures are isolated
to their worker (fall/smoking share one worker) and logged with the `[行为识别]` prefix.

## Deployment

Target: Ubuntu 22.04 x86_64, system Python 3.10. Run the explicit installer from
the `car_vel` package directory:

```bash
bash scripts/install_behavior_deps.sh
```

It installs pinned dependencies into `~/.local/share/robot-behavior/venv` and
unpacks the verified PaddleDetection archive into the adjacent `vendor` folder.
It does not install into the ROS, thermal-camera or ordinary-recognition Python
environment. `python3.10-venv` must already be installed. Package downloads need
network access. `BEHAVIOR_HOME` may select another installation location; set
the gateway's behavior Python path to its `venv/bin/python` afterward.

The installer checks CPU dependency imports, then starts an owned worker with a
90-second deadline to load all three models and validate its CPU ready response. It
does not run a frame forward or measure inference throughput during installation.
It prints optional `espeak-ng` (Chinese offline voice) and `ffmpeg` installation
commands; neither is installed with automatic sudo.

```bash
bash person_behavior/run_worker.sh
```

For direct clients, set the working directory to this `person_behavior` folder,
clear inherited `PYTHONPATH`, `PYTHONHOME`, `LD_LIBRARY_PATH`, `LD_PRELOAD`, and
launch the isolated Python with `-s -m pphuman_runtime.worker --config
config/inference.yaml`. Set `PYTHONNOUSERSITE=1`, `OMP_NUM_THREADS=2`,
`OPENBLAS_NUM_THREADS=2`, and `MKL_NUM_THREADS=2`. The launcher does this.
Do not activate this environment over the existing ROS process.

## Protocol

Every message begins with four-byte unsigned big-endian JSON byte length,
followed by UTF-8 JSON. An `infer` request immediately appends exactly
`payload_bytes` raw tightly packed uint8 BGR bytes:

```json
{"op":"infer","request_id":1,"source_epoch":1,"stamp_ns":1000000000,"shape":[480,640,3],"payload_bytes":921600,"enabled_branches":{"fall":true,"fight":true,"smoking":true}}
```

`enabled_branches` is optional. Configuration can disable a branch permanently;
a request cannot enable a configuration-disabled branch. Changes to switches,
source epoch, image shape, nonincreasing timestamp, or a stream gap reset history.
Control packets are `{"op":"reset","request_id":2,"source_epoch":2}` and
`{"op":"shutdown","request_id":3,"source_epoch":2}`, without image bytes.

The worker emits `type: ready`, then `result`, `reset_ack` or `shutdown_ack`.
Results preserve `request_id`, `source_epoch` and `stamp_ns`. Native library
diagnostics go to stderr so stdout remains exclusively binary protocol.

Fight-only gateway requests may also include `clip_stamps_ns`: 1–8 strictly
increasing nonnegative capture timestamps ending at `stamp_ns`. `shape` then
describes each frame and `payload_bytes` is the count times the BGR frame size;
payload frames appear in timestamp order. The total payload remains bounded by
25,165,824 bytes. `enabled_branches` must be exactly `fall:false,fight:true`.
The worker reconstructs the window from these real captures and independently
enforces sample gaps and total span. Overlapping sliding clips do not duplicate
frames inside a clip or reuse evidence across a reset.

- `persons[].actions.fall`: label `fall`, `lying`, `other` or null; score is
  always null, `score_kind: rule_evidence`. Static lying and an observed falling
  transition remain distinct. Insufficient pose visibility or history remains
  unknown. A person's outer `score` is the detection score, not fall confidence.
- `scene.fight`: label `fight`, `other` or null. Score is the upstream model's
  class probability, not calibrated real-world confidence. The scene result is
  not assigned to an individual. Eight real sampled frames are required.
- `pose_status`: successful observation is `state: valid` with `person_count`;
  a disabled fall branch is `state: disabled`. An empty successful pose result
  means no person was observed, not that everyone in the scene was assessed.
- `state` can be valid, unknown, warming or disabled. Never translate unknown,
  insufficient sampling, startup, disabled or expired evidence into normal.

The original .3-second fall observation gap, .6-second fight sample gap and
one-second stream reset remain enforced. No missed frames are duplicated, and
CPU slowness never stretches these thresholds. On slow hardware this may
produce little usable temporal evidence; the benchmark measures that limitation.
Pose recovery reuses the same session and may add one rotated forward on an
empty native detection with recent evidence, increasing latency.

## File-video benchmark

```bash
bash scripts/benchmark_behavior.sh --video /absolute/path/labeled-video.mp4 \
  --output /tmp/behavior-benchmark.json --max-seconds 20 --max-frames 120
```

The default realtime replay drops frames missed while CPU inference is busy,
preserving actual media timestamps. `--mode offline` evaluates each decoded
frame with source timestamps and separately reports CPU durations; it is not
a realtime claim. `--branches fall` or `--branches fight` isolates a branch.
The run has a 180-second total deadline, 60-second startup deadline and
30-second request deadline by default. It never opens a camera, ROS service or
robot control interface. Width is capped at 1280 unless changed explicitly.

The JSON report includes startup time, mean/p50/p95/max inference durations,
capture gaps, dropped frames, branch valid/unknown/warming/disabled counts,
unknown reasons and the last fight window. Fall counts are person observations
plus no-person frames; fight counts are scene frames. Coverage is not accuracy.
The neighboring `.worker.log` retains worker diagnostics. Error/timeout reports
return a nonzero status and retain partial measurements.

## Verification scope

The local Windows integration tests use a fake model backend to verify protocol,
CPU-only selective model construction, branch gating, time gaps, source reset, pose
visibility and score semantics. Asset integrity was verified locally. Linux
CPU model loading, throughput, labeled-video effectiveness and coexistence
with the full robot workload still need the bounded benchmark on the target
machine. No camera or robot motion validation is performed by these tests.

## Deploying the capture-starvation fix

Sync the entire updated `car_vel` source (gateway AND `person_behavior` worker)
to the small PC, including the newly bundled PPYOLOE smoking model. Existing
CPU dependencies can be reused; load-check all three models using
`../BEHAVIOR_RECOGNITION.md`. Stop the backend,
then rebuild and restart using the usual launcher:

```bash
cd /home/a/3dnav
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select car_vel lzp_service
source /home/a/3dnav/install/setup.bash
```

Refresh the recognition page, enable the required branches and start recognition.
With real camera video, inspect the following while keeping the robot stopped:

```bash
curl -sS --max-time 5 http://127.0.0.1:8766/api/behavior/status | python3 -m json.tool --no-ensure-ascii
```

Expect both worker `running` values true, eight real fight samples after roughly
two seconds of continuous capture, and fresh fight results (`valid` for a
sufficient-confidence class, `unknown` for uncertain model output). A `warming`
window below eight indicates insufficient capture continuity. `stale_results`
increasing indicates the fight process itself is too slow for the one-second
result deadline; measure it on the target host rather than increasing the
deadline or calling uncertain/expired results normal. Initialization may take
longer than the capture window. This fix is not an accuracy certification.
