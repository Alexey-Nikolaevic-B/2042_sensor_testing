# 2042 Sensor Testing Integration Guide

## Scope
- This worktree is the integration branch based on `perestroika`.
- Primary goal of this branch:
  - preserve the existing `perestroika` UI,
  - preserve `perestroika` RFID behavior,
  - integrate camera and tactile code/assets from the working sensor branch.
- Local macOS is for code audit, edits, diffing, git work, and static checks.
- Runtime truth is still Ubuntu with ROS Noetic + Gazebo Classic.

## Current Branch Identity
- Branch:
  - `codex/perestroika-camera-tactile-integration`
- Worktree:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing_perestroika_integration`
- Base branch:
  - `origin/perestroika`

## Current Integration State
- `perestroika` remains the source of truth for:
  - UI entrypoint in `__main__.py`
  - `front/`
  - RFID functionality and its worlds/assets
- Camera files have already been transferred into this branch:
  - camera SDF assets under `resources/sensors/camera`
  - camera worlds under `resources/worlds/`
  - camera modules under `src/sensors`
- Tactile files have already been transferred into this branch:
  - tactile SDF assets under `resources/sensors/tactile`
  - tactile worlds under `resources/worlds/tactile`
  - tactile modules under `src/sensors`
- Shared backend files were also updated during integration:
  - `config.py`
  - `src/core.py`
  - `src/gazebo_simulator.py`
  - `src/sensors/__init__.py`
  - `src/sensors/sensor.py`

## Important Reality Check
- File transfer is already done for camera and tactile.
- This does NOT yet mean the integration is runtime-verified.
- Ubuntu validation for this branch has not yet been completed.
- The biggest unverified shared-risk area is:
  - `src/gazebo_simulator.py`
  - `src/core.py`
- Camera/tactile assets exist in this branch, but UI visibility may still depend on the SQLite/database layer.

## Repository Locations
- Original local working copy:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing`
- Integration worktree:
  - `/Users/arturkuanyshev/Projects/2042_sensor_testing_perestroika_integration`
- Ubuntu runtime repo:
  - `/home/artur/Documents/2042_sensor_testing`

## Secrets And Access
- Ubuntu SSH target:
  - `artur@100.89.63.4`
- Do not commit passwords, tokens, or plaintext credentials.
- If Ubuntu is offline, continue with local static integration work only.

## Branch Workflow
- Use `codex/...` branches only.
- For this integration branch, prefer narrow commits by subsystem:
  - camera transfer
  - tactile transfer
  - DB/UI integration
  - Ubuntu validation follow-ups
- Do not rewrite history of shared branches unless explicitly asked.
- Do not revert unrelated user changes.

Useful commands:
```bash
git status --short --branch
git log --oneline --decorate -5
git diff --stat origin/perestroika..HEAD
git push origin codex/perestroika-camera-tactile-integration
```

## What Must Be Preserved
- Preserve `perestroika` UI entrypoint in:
  - `__main__.py`
- Preserve `front/`
- Preserve RFID code and RFID worlds/assets already present in `perestroika`
- Do NOT pull RFID behavior from the tactile working branch

## Cameras In This Branch
- Camera assets currently present:
  - `resources/sensors/camera/*.sdf`
- Camera worlds currently present:
  - `resources/worlds/camera_*.world`
  - `resources/worlds/camera_depth_perception.world`
  - `resources/worlds/depth_camera/depth_perception_test.world`
- Camera code currently present:
  - `src/sensors/mono_camera.py`
  - `src/sensors/depth_camera.py`
  - `src/sensors/mono_profile_base.py`
  - `src/sensors/depth_profile_base.py`
  - `src/sensors/stereo_profile_base.py`
  - `src/sensors/camera_sdf_registry.py`
  - `src/sensors/uvc_profile_640x480_60deg.py`
  - `src/sensors/sdf_profile.py`

## Tactile In This Branch
- Tactile assets currently present:
  - `resources/sensors/tactile/*/model.sdf`
  - `resources/sensors/tactile/*/model.config`
- Tactile worlds currently present:
  - `resources/worlds/tactile/tactile_t1_threshold.world`
  - `resources/worlds/tactile/tactile_t2_uniformity.world`
  - `resources/worlds/tactile/tactile_t3_stability.world`
  - `resources/worlds/tactile/tactile_t4_peak_load.world`
- Tactile code currently present:
  - `src/sensors/tactile_profile_base.py`
  - `src/sensors/tactile_harness.py`
  - `src/sensors/tactile_specs.py`
  - `src/sensors/tactile_sdf_registry.py`

## RFID In This Branch
- RFID should be treated as inherited from `perestroika`.
- Do not replace RFID with code from the tactile working branch.
- If debugging RFID regressions here, first suspect shared backend integration before changing RFID itself.

## Integration Risks To Audit First
1. `src/core.py`
2. `src/gazebo_simulator.py`
3. `src/sensors/__init__.py`
4. `src/sensors/sensor.py`
5. DB/UI visibility path:
   - `src/database/`
   - `front/logic_sensor_repository.py`

## Local Validation Guidance
- Static validation on macOS is acceptable:
```bash
python3 -m compileall config.py src
```
- Good local audit commands:
```bash
git diff --stat origin/perestroika..HEAD
git diff --name-status origin/perestroika..HEAD
rg -n "camera|tactile|rfid" src resources front
```

## Ubuntu Runtime Workflow
- When Ubuntu is available, always source ROS first:
```bash
source /opt/ros/noetic/setup.bash
source /home/artur/Documents/2042_sensor_testing/catkin_ws/devel/setup.bash
cd /home/artur/Documents/2042_sensor_testing
```

## Recommended Ubuntu Validation Order
1. Camera smoke:
```bash
python3 __main__.py --sensor-type camera --sensor-name axis_wide_110deg --suite
```
2. RFID smoke using the existing `perestroika` path
3. Tactile smoke on one sensor:
```bash
python3 __main__.py --sensor-type tactile --sensor-name ati_nano17 --suite
```
4. Only after that:
```bash
python3 __main__.py --sensor-type tactile --all-sensors --suite
```

## Known Unknowns
- Whether UI automatically exposes all transferred camera and tactile sensors through the DB layer
- Whether the integrated `src/gazebo_simulator.py` is fully compatible with `perestroika` UI workflows
- Whether all tactile tests still behave correctly in this integrated branch on Ubuntu

## Preferred Output Style For Future Sessions
- Clearly separate:
  - what is already transferred,
  - what is only statically verified,
  - what is runtime-verified,
  - what remains uncertain
- If something is an inference, label it as an inference.
- When reporting risk, name the exact file.
