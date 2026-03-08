# 2042 Sensor Testing Agent Guide

## Scope
- This repository is primarily validated on Ubuntu with ROS Noetic + Gazebo Classic.
- Local macOS is suitable for reading/editing code, git work, and static analysis.
- Runtime validation for camera tests must be done on the Ubuntu machine.
- Unless the user explicitly asks otherwise, prefer narrow fixes over broad refactors.
- Unless the user explicitly asks otherwise, do not touch RFID when working on camera tests.

## Repository Locations
- Local macOS working copy:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing`
- Ubuntu runtime working copy:
  - `/home/artur/Documents/2042_sensor_testing`
- A second Ubuntu copy may also exist under `Downloads`, but the verified runtime repo is:
  - `/home/artur/Documents/2042_sensor_testing`

## Secrets And Access
- Ubuntu SSH target:
  - `artur@100.89.63.4`
- Do not commit passwords, tokens, or plaintext credentials into the repository.
- If SSH or GitHub credentials are needed, obtain them from a secure out-of-band source already available to the user.
- It is acceptable to document hostnames, usernames, paths, and operational steps in this file.

## Branch And Git Workflow
- Use `codex/...` branches.
- Keep changes scoped to the user’s requested tests.
- Commit after each meaningful logical step.
- Push to `origin` after local verification.
- Do not revert unrelated user changes.
- Ignore local `.DS_Store` noise unless the user explicitly wants cleanup.

Recommended flow:
1. Audit the target tests locally.
2. Form a concrete hypothesis.
3. Make the smallest viable fix.
4. Run static validation locally.
5. Commit locally.
6. Push to `origin`.
7. Update the Ubuntu repo and run runtime validation there.
8. Report exact outcomes with concrete metrics.

Useful commands:
```bash
git status --short --branch
git checkout -b codex/<task-name>
git add <files>
git commit -m "<message>"
git push origin codex/<task-name>
```

## Ubuntu Runtime Workflow
- Always source ROS before running tests:
```bash
source /opt/ros/noetic/setup.bash
source /home/artur/Documents/2042_sensor_testing/catkin_ws/devel/setup.bash
cd /home/artur/Documents/2042_sensor_testing
```

- Single-test camera run:
```bash
python3 __main__.py --sensor-type camera --sensor-name axis_wide_110deg --test c4_geometries_presence_test
```

- Typical mono sensors used during validation:
  - `axis_wide_110deg`
  - `picam3_std`

- Read latest report:
```bash
python3 - <<'PY'
import json, pathlib
p = pathlib.Path("results/axis_wide_110deg/report.json")
print(json.dumps(json.loads(p.read_text()), ensure_ascii=False, indent=2))
PY
```

## Updating Ubuntu When `git pull` Is Blocked
- The Ubuntu repo may have `origin` configured as HTTPS:
  - `https://github.com/Alexey-Nikolaevic-B/2042_sensor_testing.git`
- If `git fetch` or `git pull` requires unavailable GitHub credentials, use a bundle fallback.

Local macOS:
```bash
git bundle create /tmp/codex-update.bundle codex/<task-branch>
scp /tmp/codex-update.bundle artur@100.89.63.4:/tmp/codex-update.bundle
```

Ubuntu:
```bash
cd /home/artur/Documents/2042_sensor_testing
git pull --ff-only /tmp/codex-update.bundle codex/<task-branch>
```

This preserves branch history without changing the Ubuntu remote configuration.

## Primary Code Areas
- Mono camera test logic:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing/src/sensors/mono_profile_base.py`
- Mono camera sensor definitions:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing/resources/sensors/camera`
- Gazebo worlds:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing/resources/worlds`
- Core reporting/runtime:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing/src/core.py`
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing/src/gazebo_simulator.py`

## Required Diagnostic Sequence For Camera Tests
For each target test, inspect the full execution path:
1. Which world is opened.
2. Whether `open_scene()` actually succeeds.
3. Which Gazebo services are available.
4. Whether `/clock` is alive.
5. Which image topic is expected.
6. Which image topic is actually resolved at runtime.
7. Whether an image frame is really received.
8. If the test moves an object, whether a fresh frame is received after movement.
9. How the metric is computed from the frame.
10. How `PASS` or `FAIL` is decided.
11. What lands in `report.json`.
12. What lands in `results/<sensor>/metrics/*.json`.

## Camera Test Methodology

### General Rules
- Prefer runtime-resolved topics over hardcoded `self.IMAGE_TOPIC` when the scene resolver already provides a mapping.
- A failure to open the scene, receive an image, or compute a metric must not degrade into a false `PASS`.
- For movement-based tests, require a fresh image after `set_pose()` or `set_model_state()`.
- Do not tune thresholds before validating that the world geometry and the test geometry agree.
- If Gazebo startup fails, classify whether the root cause is:
  - display/rendering
  - missing Gazebo API services
  - bad world geometry
  - topic readiness mismatch

### Runtime Image Rules
- Use display fallback when plain SSH sessions do not provide `DISPLAY`.
- Prefer `_resolved_image_topic()` to get the actual runtime topic.
- Prefer `_wait_image_after(prev_stamp_s, topic=...)` when the object moves between measurements.
- If a frame is missing, raise with topic and stage context.

### Reporting Rules
- `report.json` is intentionally the last-run summary only.
- Detailed per-test debugging belongs in:
  - `results/<sensor>/metrics/*.json`
- Diagnostics should include when relevant:
  - `world_file`
  - `expected_topic`
  - `resolved_topic`
  - `scene_open_success`
  - `status`
  - `error_reason`
  - any test-specific numeric metrics

## Test-Specific Metrics That Have Worked Well

### C1 Size Order
- Compare perceived object size at increasing distances.
- Useful metrics:
  - bounding-box area
  - red pixel counts
  - object positions
- Critical requirement:
  - the object trajectory in code must match the trajectory implied by the world layout

### C2 Resolution
- Use `sensor_msgs/Image.width` and `sensor_msgs/Image.height`.
- Compare directly against camera SDF expectations.
- Do not infer resolution indirectly from resized debug frames.

### C4 Geometries Presence
- Count color-specific pixels for expected colored objects:
  - red
  - green
  - blue
  - yellow
- World startup matters as much as the pixel detector.
- If Gazebo exposes only:
  - `/gazebo/get_loggers`
  - `/gazebo/set_logger_level`
  then the world likely stalled before full Gazebo ROS API initialization.

### C10 Clipping
- The target object must be physically suitable for clipping measurement.
- Near/far checks should be tied to visibility boundaries, not stale frames.
- If the target is thick along camera depth, near clipping is biased.
- If the target is too small in the image, far clipping is biased by the detector threshold rather than clip distance.
- For this repository, a thin target along X with larger Y/Z extent has worked well.

### C11 FPS Stability
- Use header timestamps from the image stream.
- Use monotonic timestamps only.
- Report:
  - raw captured frames
  - monotonic frames
  - dropped non-monotonic timestamps
  - first and last stamp
  - measured FPS
  - jitter
- FPS should be computed using frame intervals, not optimistic overcounting.

## Validated Mono Fixes Already Landed
These tests were fixed and confirmed on Ubuntu runtime:
- `c1_size_order_test`
- `c2_resolution_test`
- `c4_geometries_presence_test`
- `c10_clipping_test`
- `c11_fps_stability_test`

Not yet part of that confirmed set in this workstream:
- `c7_occlusion_test`
- `c9_fov_test`

## Known World-Specific Findings
- `camera_c4_geometries.world`
  - Primitive cone geometry caused Gazebo Classic on the Ubuntu runner to stall during startup.
  - Symptom:
    - only logger services appear under `/gazebo/*`
  - A stable yellow box replacement allowed full Gazebo API startup.

- `camera_c10_clipping.world`
  - The clipping target must be designed to measure clip planes, not detector weakness.
  - A thin target along X avoids near-plane depth bias.
  - A larger Y/Z face keeps the object detectable out to the far clip.

## How To Run A Tight Audit Before Fixing
1. Read the test function in `mono_profile_base.py`.
2. Read its world file.
3. Read the camera SDF for the target sensor.
4. Run the test on Ubuntu.
5. Inspect:
   - `report.json`
   - metrics JSON
   - Gazebo services
   - resolved topics
6. Decide whether the bug is in:
   - the test logic
   - the world
   - the sensor SDF assumptions
   - startup/rendering
7. Only then patch the narrowest layer that fixes the root cause.

## Preferred Output Style For Future Sessions
- Be explicit about:
  - what was audited
  - what actually failed
  - whether the failure is startup, topic, frame, metric, or reporting
- When fixing:
  - name the root cause
  - name the exact files changed
  - include the Ubuntu validation results
- Use concrete values:
  - exact distances
  - exact FPS
  - exact scene reasons
  - exact commit hashes

## Short Prompt Template For Future Chats
Use this when starting a new Codex thread in this repo:

```text
You are working in 2042_sensor_testing.

Target only the specified mono camera test(s). Do not touch RFID unless explicitly asked.

Work in this order:
1. audit code + world/SDF
2. confirm the failure on Ubuntu runtime
3. identify the real root cause
4. make only narrow fixes
5. commit/push in a codex/... branch
6. update Ubuntu repo
7. rerun tests and report exact metrics

Important:
- macOS is for editing only
- runtime truth is Ubuntu at /home/artur/Documents/2042_sensor_testing
- check scene open, Gazebo services, resolved topics, image receipt, fresh frames after movement, metric logic, report.json, and metrics/*.json
```
