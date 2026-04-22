# ChatGPT Handoff Notes

Last updated: 2026-04-22

Purpose:
- Provide a compact repository-grounded handoff for external analysis in ChatGPT.
- Focus analysis on AprilTag-based triggering, coarse-position entry, and state-machine structure.

## Repository Scope

- Workspace root:
  - `/home/xiang/ros2_ws`
- Main repository:
  - `/home/xiang/ros2_ws/src/jaka_a5`

## Current Runtime Architecture

- Gazebo simulation publishes the eye-in-hand camera image.
- `camera_qos_relay.py` repackages camera QoS for downstream nodes.
- `apriltag_ros` publishes AprilTag detections and TF.
- `apriltag_subscriber.py` converts TF into semantic topics:
  - `/target_pose`
  - `/tag_pose_camera`
  - `/tag_pose_world`
  - `/battery_center`
- `vs_controller.py` owns the main task state machine and visual-servo logic.
- `pregrasp_coordinator.py` receives `/task_frames/pre_grasp`, sends a MoveIt coarse-positioning goal, then publishes `/visual_servo/enable`.
- `integration.launch.py` brings up Gazebo, ros2_control, MoveIt, and the vision stack together.

## Files That Matter Most

- `jaka_a5_vision/jaka_a5_vision/apriltag_subscriber.py`
- `jaka_a5_vision/jaka_a5_vision/vs_controller.py`
- `jaka_a5_vision/jaka_a5_vision/pregrasp_coordinator.py`
- `jaka_a5_vision/jaka_a5_vision/experiment_logger.py`
- `jaka_a5_vision/launch/vs.launch.py`
- `../jaka_a5_bringup/launch/integration.launch.py`

## Current State Machine

`vs_controller.py` currently implements:

- `S0_STANDBY`
- `S1_SIDE_APPROACH`
- `S2_ALIGN`
- `S3_CLAMP`
- `S4_EXTRACT`
- `S5_SAFE_EXIT`
- `S6_PLACE_OLD_BATTERY`
- `S7_TOOL_RESET`
- `S8_GRASP_NEW_BATTERY`
- `S9_MOVE_TO_DOCK`
- `S10_INSERT_NEW_BATTERY`
- `S11_RESET`

Current intended split:

- `S0`: standby + search
- `S1`: MoveIt coarse positioning to `pre_grasp`
- `S2`: visual-servo fine alignment
- `S3-S11`: task-level simulated battery-swap flow

## Key Changes Already Made

- Added CSV event logging through `experiment_logger.py`.
- Added Chinese terminal feedback for:
  - detected tag pose relative to camera and tool
  - state transitions
  - MoveIt coarse-position send/success/failure
- Search behavior changed from joint swinging to bounded camera-direction-preserving spiral search.
- Search triggering logic was relaxed:
  - before: target lock required minimum sweep steps
  - now: if the tag stays stably visible for a configured duration, the controller bypasses the minimum-sweep gate and enters coarse positioning
- `pre_grasp` and `grasp_target` are frozen once the target is confirmed.
- `pregrasp_coordinator.py` retries MoveIt with backoff instead of spamming requests.

## Current Verified Behavior

In a recent validation run:

- The terminal printed repeated Chinese target-pose feedback:
  - `检测到Tag: 相机坐标系 xyz=(...), 末端坐标系 xyz=(...)`
- Then the controller printed:
  - `Tag 持续稳定可见，跳过最低扫描步数限制，直接进入粗定位`
  - `状态切换: s0_standby -> s1_side_approach (...)`
- `pregrasp_coordinator.py` printed:
  - `发送 MoveIt 粗定位目标到 pre_grasp`
  - `MoveIt 粗定位成功（plan-and-execute），开始使能视觉伺服`

Matching CSV evidence exists in:

- `/tmp/jaka_a5_cn_verify_log.csv`

Relevant event chain from that log:

- `target_confirmation_bypass`
- `target_confirmed`
- `cycle_started`
- `state_exit` from `s0_standby`
- `state_enter` into `s1_side_approach`
- `moveit_goal_sent`
- `moveit_success`
- `servo_gate_changed` to enabled

## Main Open Problems For Analysis

### 1. AprilTag visible but not always entering coarse positioning

Even after improvements, this area still needs deeper review.

Questions to analyze:

- Is the target-confirmation logic still too dependent on timing artifacts?
- Is `apriltag_subscriber.py` still dropping useful detections due to stale/future TF filtering?
- Should the lock decision use a stronger semantic rule than:
  - consecutive fresh cycles
  - minimum sweep steps
  - stable-visible-duration bypass
- Should the system explicitly detect:
  - "tag already centered and sufficiently large/stable in the initial image"
  and skip most search behavior entirely?

### 2. State-machine structure and responsibility split

Questions to analyze:

- Is `vs_controller.py` still carrying too many responsibilities?
- Should `S1` and later coarse waypoint states be managed by a separate task coordinator instead of the servo node?
- Is the current `S0 -> S1 -> S2 -> S3...` transition structure technically clean and thesis-friendly?
- Which states should be:
  - MoveIt-driven
  - visual-servo-driven
  - pure process/task execution
- What failure and timeout transitions are still missing or too implicit?

### 3. Servo enable handshake

There are runs where MoveIt succeeds first and the servo-enable update becomes visible shortly after.

Questions to analyze:

- Is the `/visual_servo/enable` transient-local Bool approach robust enough?
- Should the handshake become an explicit stateful coordinator protocol instead of a single Bool topic?
- Would an action/service/state topic split make the coarse-to-fine transition more observable and deterministic?

## Recommended Reproduction Path

From a clean terminal:

```bash
cd /home/xiang/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jaka_a5_bringup integration.launch.py \
  enable_servo_controller:=true \
  enable_moveit_coordinator:=true \
  event_log_path:=/tmp/jaka_a5_cn_verify_log.csv
```

If old processes may still be alive:

```bash
pkill -f 'ros2 launch jaka_a5_bringup integration.launch.py|ign gazebo|gz sim|ros_gz_bridge/parameter_bridge|ros_gz_image/image_bridge|jaka_a5_vision/camera_qos_relay|apriltag_ros/apriltag_node|jaka_a5_vision/apriltag_subscriber|jaka_a5_vision/vs_controller|jaka_a5_vision/pregrasp_coordinator|moveit_ros_move_group/move_group|rviz2/rviz2'
```

## Prompt For External ChatGPT Analysis

Use this prompt after giving ChatGPT the repository or the relevant files.

```text
Please deeply analyze this ROS 2 / MoveIt / Gazebo project, focusing on two areas:

1. AprilTag localization and trigger logic, especially the failure mode where a complete AprilTag is already visible in the end-effector camera image, but the system does not enter coarse positioning (`S1_SIDE_APPROACH`) quickly or reliably.

2. The structure of the visual-servo task state machine in `vs_controller.py`, including whether responsibilities are split cleanly between:
   - AprilTag perception
   - coarse positioning by MoveIt
   - fine alignment by visual servo
   - later battery-swap task states

Please base your reasoning on the actual repository code rather than generic robotics advice.

Key files to inspect first:
- `jaka_a5_vision/jaka_a5_vision/apriltag_subscriber.py`
- `jaka_a5_vision/jaka_a5_vision/vs_controller.py`
- `jaka_a5_vision/jaka_a5_vision/pregrasp_coordinator.py`
- `jaka_a5_vision/jaka_a5_vision/experiment_logger.py`
- `jaka_a5_vision/launch/vs.launch.py`
- `jaka_a5_bringup/launch/integration.launch.py`

Please answer in this structure:

1. Runtime dataflow summary
2. Exact trigger chain from AprilTag detection to `S1_SIDE_APPROACH`
3. Why a visible tag might still fail to trigger coarse positioning
4. Weak points in stale-TF filtering, target confirmation, and search bypass logic
5. State-machine architecture critique
6. Specific code-level refactor proposals
7. A prioritized fix plan with the minimum risky changes first

Please be concrete about:
- topic names
- frame semantics
- transition conditions
- timing dependencies
- hidden coupling between nodes
- whether the current `servo_enable` handshake is robust

If you infer behavior, label it clearly as inference.
If you recommend changes, point to the exact functions or code regions that should change.
```
