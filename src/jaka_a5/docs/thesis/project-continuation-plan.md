# JAKA A5 Project Continuation Plan

Last updated: 2026-04-21

Purpose:
- Record the current engineering judgment and next execution steps so future work can resume directly from the repository state.

## Current Judgment

- The repository is no longer in a "from scratch" state.
- The overall architecture is already in place:
  - `jaka_a5_description`
  - `jaka_a5_gazebo`
  - `jaka_a5_control`
  - `jaka_a5_planning`
  - `jaka_a5_vision`
  - `jaka_a5_bringup`
- The shortest path to completion is not a redesign.
- The main remaining work is to close the engineering loop around:
  - task-frame semantics
  - MoveIt coarse positioning vs visual-servo fine alignment
  - experiment logging and thesis/code consistency

## Progress Update

- Priority 1 is partially implemented in the repository code.
- `apriltag_subscriber.py` now publishes semantic observation topics:
  - `/tag_pose_camera`
  - `/tag_pose_world`
  - `/battery_center`
- Backward compatibility is preserved through `/target_pose`.
- `vs_controller.py` now publishes inspectable task-goal topics:
  - `/task_frames/pre_grasp`
  - `/task_frames/grasp_target`
  - `/task_frames/place_target`
- The controller now defaults to consuming `/tag_pose_camera` instead of the ambiguous `/target_pose`.
- Remaining Priority 1 work is runtime validation in RViz and TF inspection, not basic code plumbing.
- Priority 2 is partially implemented with a minimal staged workflow:
  - `pregrasp_coordinator.py` subscribes to `/task_frames/pre_grasp`
  - the coordinator sends a MoveIt `MoveGroup` goal for coarse positioning
  - after successful coarse positioning, it publishes `/visual_servo/enable`
  - `vs_controller.py` can now wait for that external enable signal before starting fine alignment
- Fine alignment semantics are now explicit:
  - `ALIGN` means "keep the tag centered while moving to the final grasp pose"
  - when the tag is centered and the camera-to-tag distance reaches 3 cm, the controller transitions directly to `GRASP`
  - `GRASP -> LIFT -> TRANSFER -> PLACE -> RETREAT` continues the battery-swap task state machine
- The staged launch path is now available through:
  - `integration.launch.py`
  - `vs.launch.py`
  - launch argument: `enable_moveit_coordinator:=true`

## Scope Freeze

The project should be finished under these explicit assumptions:

- Known simulated camera extrinsics from URDF; no real hand-eye calibration solving is required.
- The controller should be described as pose-based or hybrid visual servoing driven by AprilTag 3D pose.
- Do not claim a pure image-based visual servoing implementation if the code is using 3D tag pose and IK.
- Grasp/transfer/place is currently a task-level simulated attachment workflow in Gazebo.
- Do not describe the current implementation as real contact grasping.

## Current System Summary

- Eye-in-hand camera is mounted on `J6`.
- Gazebo publishes:
  - `/camera/image_raw`
  - `/camera/camera_info`
- `camera_qos_relay.py` republishes camera topics for the vision stack.
- `apriltag_ros` provides tag detection and TF.
- `apriltag_subscriber.py` currently converts tag TF into `/target_pose`.
- `vs_controller.py` contains the main task state machine:
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
- `integration.launch.py` brings up Gazebo, ros2_control, MoveIt, and vision.

## State Machine Configuration Guidance

- Design the state machine from the real battery-swap procedure first, not from existing variable names.
- Target workflow should now be treated as this 12-state loop:
  - `S0` standby
  - `S1` side approach (coarse positioning)
  - `S2` alignment (fine positioning)
  - `S3` clamp old battery
  - `S4` extract old battery
  - `S5` safe exit
  - `S6` place old battery
  - `S7` end-effector reset
  - `S8` grasp new battery
  - `S9` move to docking pose
  - `S10` insert new battery
  - `S11` reset
  - then return to `S0`
- For each state, define explicitly:
  - target pose or path
  - reference frame
  - transition condition
  - failure/timeout condition
- Recommended path split for this workflow:
  - `S1`, `S6`, `S8`, `S9`, `S11`: MoveIt waypoint / coarse motion states
  - `S2`: vision-servo fine alignment state
  - `S4`, `S5`, `S10`: process-axis linear motion states
  - `S3`: clamp/attach execution state
- Current fine-alignment rule remains:
  - `S2` should keep the tag centered while converging to the final grasp pose
  - when the tag is centered and camera-to-tag distance reaches 3 cm, transition to clamp/grasp
- Recommended real-process pose/offset set to define explicitly:
  - `approach_pose_old_battery`
  - `extract_distance`, `extract_axis`
  - `safe_exit_pose`
  - `old_battery_drop_pose`
  - `reset_pose_after_old_battery`
  - `new_battery_pick_pose`
  - `dock_pose`
  - `insert_distance`, `insert_axis`
  - `home_pose` or `standby_pose`
- The current code now uses a 12-state skeleton in `vs_controller.py`, but later task states are still implemented with task-level simulation offsets rather than full real-process geometry and dual-battery world modeling.

## Most Important Remaining Problems

### 1. Task-frame semantics are still implicit

Current risk:
- `/target_pose` is effectively "tag pose in camera frame", but the task-level meanings of:
  - `tag_pose_camera`
  - `tag_pose_world`
  - `battery_center`
  - `pre_grasp`
  - `grasp_target`
  - `place_target`
  are not made explicit enough in TF/topics.

Required outcome:
- These task frames must become visible, inspectable, and documented.

### 2. Coarse planning and fine alignment are not cleanly separated

Current risk:
- `vs_controller.py` currently carries too much of the end-to-end behavior.
- The intended thesis narrative is cleaner if:
  - MoveIt handles coarse positioning to `pre_grasp`
  - visual servo handles only the final alignment and approach

Required outcome:
- Introduce a minimal coordinator or staged workflow instead of a full rewrite.

### 3. Experimental evidence is too weak

Current risk:
- The codebase can demonstrate behavior, but the thesis needs measurable results.
- Current documentation and code contain mismatches in:
  - frame naming
  - camera update rates
  - controller timing
  - timeout values
  - state machine naming
  - claims about validation and performance

Required outcome:
- Replace placeholder or optimistic thesis claims with measured data from the actual repo state.

## Priority Order For Next Work Session

### Priority 1: Make task frames explicit

Target files:
- `src/jaka_a5/jaka_a5_vision/jaka_a5_vision/apriltag_subscriber.py`
- `src/jaka_a5/jaka_a5_vision/jaka_a5_vision/vs_controller.py`

Goal:
- Publish or rename task-relevant outputs so the system explicitly exposes:
  - `tag_pose_camera`
  - `tag_pose_world`
  - `pre_grasp`
  - `grasp_target`
  - `place_target`

Validation:
- RViz TF tree is interpretable.
- `tf2_echo` and topic inspection match the intended task semantics.

### Priority 2: Split MoveIt coarse motion from servo fine alignment

Target files:
- `src/jaka_a5_bringup/launch/integration.launch.py`
- `src/jaka_a5/jaka_a5_planning/launch/move_group.launch.py`
- `src/jaka_a5/jaka_a5_vision/jaka_a5_vision/vs_controller.py`
- possibly a new small coordinator node

Goal:
- Implement:
  1. ready/home
  2. MoveIt to `pre_grasp`
  3. visual servo for final alignment
  4. existing attach/lift/transfer/place flow

Validation:
- The architecture can be clearly explained as:
  - coarse planning by MoveIt
  - fine correction by visual servoing

### Priority 3: Add logging and failure reporting

Target files:
- `src/jaka_a5/jaka_a5_vision/jaka_a5_vision/vs_controller.py`
- possibly a new metrics/logging helper

Minimum required signals:
- current state
- target freshness
- IK failure
- trajectory timeout
- attach/set_pose failure
- per-stage timing
- final success/failure reason

Desired artifact:
- CSV, rosbag, or another simple machine-readable experiment log.

## Recommended Minimal Thesis Claims

Use wording close to the actual implementation:

- "Eye-in-hand camera with known simulated extrinsics"
- "AprilTag-based pose estimation through TF"
- "Pose-based visual servoing / hybrid visual servoing"
- "MoveIt coarse positioning with visual fine alignment"
- "Task-level simulated battery replacement using Gazebo attachment-style object following"

Avoid unsupported wording:

- "Completed real hand-eye calibration"
- "Pure IBVS" if 3D pose and IK are the actual control basis
- "Real contact grasping" for the current Gazebo workflow

## Minimum Viable Completion Plan

### Days 1-2

- Clean up and expose task-frame semantics.
- Verify the TF chain end to end.

### Days 3-4

- Add the smallest possible coordinator for:
  - MoveIt to `pre_grasp`
  - servo for final alignment

### Days 5-6

- Run a complete pick-transfer-place cycle once with the existing task state machine.

### Days 7-8

- Add logging for:
  - tag loss
  - IK failure
  - trajectory timeout
  - set-pose failure
  - total task duration

### Days 9-10

- Run 10-20 experiments and replace thesis placeholders with measured data.

### Days 11-14

- Freeze implementation.
- Align thesis text with real code behavior.
- Prepare figures, TF tree, flowchart, screenshots, and demo material.

## Resume Instruction

When resuming from this repository in a future session:

1. Read this file first.
2. Then inspect:
   - `src/jaka_a5_bringup/launch/integration.launch.py`
   - `src/jaka_a5/jaka_a5_vision/jaka_a5_vision/apriltag_subscriber.py`
   - `src/jaka_a5/jaka_a5_vision/jaka_a5_vision/vs_controller.py`
   - `src/jaka_a5/docs/thesis/thesis-draft.md`
3. Start with Priority 1 unless newer commits already completed it.
