# JAKA A5 End-Effector Camera Setup and Validation

This document records the camera setup that was validated on April 20, 2026 for this project:

- ROS 2: Humble
- Gazebo Sim: Fortress / ignition-gazebo6
- Robot: JAKA A5
- Workspace: `/home/xiang/ros2_ws`

The goal of this tutorial is narrow and practical:

1. Attach a camera to the robot end effector in URDF / xacro.
2. Make Gazebo Sim publish the camera image and camera info.
3. Bridge the image into ROS 2 using the official `ros_gz_image` workflow.
4. Verify that the image topic is really alive and not black.

## 1. Official Reference Pattern

This setup follows the same idea used by the official Humble / Fortress examples:

- Gazebo camera sensor publishes a Gazebo image topic.
- `ros_gz_image image_bridge` bridges the image into ROS 2.
- `ros_gz_bridge parameter_bridge` bridges `CameraInfo`.

Reference material:

- Gazebo migration guide:
  `https://github.com/gazebosim/docs/blob/master/migrating_gazebo_classic_ros2_packages.md`
- ROS Gazebo demos:
  `/opt/ros/humble/share/ros_gz_sim_demos/launch/camera.launch.py`
- ROS Gazebo image bridge demo:
  `/opt/ros/humble/share/ros_gz_sim_demos/launch/image_bridge.launch.py`

## 2. Files Used In This Project

The camera path in this repository is split across these files:

- Camera mount and model:
  [jaka_a5.robot.xacro](/home/xiang/ros2_ws/src/jaka_a5/jaka_a5_description/urdf/jaka_a5.robot.xacro)
- Gazebo camera sensor:
  [gazebo.urdf.xacro](/home/xiang/ros2_ws/src/jaka_a5/jaka_a5_description/urdf/gazebo.urdf.xacro)
- Gazebo launch and ROS bridge:
  [sim.launch.py](/home/xiang/ros2_ws/src/jaka_a5/jaka_a5_gazebo/launch/sim.launch.py)

## 3. What Was Changed

### 3.0 Add an `ImageDisplay` panel to the Gazebo world

Why:

- The official camera demos do not only publish image topics.
- They also show the live camera feed directly inside the Gazebo GUI.

What changed:

- [battery_station.world](/home/xiang/ros2_ws/src/jaka_a5/jaka_a5_gazebo/worlds/battery_station.world) now contains an `ImageDisplay` plugin bound to `/camera/image_raw`.

### 3.1 Add a visible camera body to the end effector

Why:

- An empty `camera_link` does not render in Gazebo.
- If there is no `visual`, the camera seems to be “missing” even if the sensor exists.

What changed:

- `camera_link` now has a small box body and a cylindrical lens.
- This makes the end-effector camera visible in Gazebo.

### 3.2 Mount the camera with a fixed joint on `J6`

Why:

- The camera must move with the robot wrist.
- The wrist frame orientation does not automatically point toward the workstation.

What changed:

- `camera_joint` is fixed to `J6`.
- The camera was rotated by `rpy="0 0 3.14159"` so the sensor faces the workstation side instead of looking away from it.
- After rotating the camera, the mount offset was also moved to `xyz="-0.055 0 0.045"` so the lens stays on the real front side of the housing instead of ending up toward `J6`.

### 3.3 Move the sensor slightly forward from the camera body

Why:

- If the sensor origin stays inside the camera housing, Gazebo can render the housing itself and the image may become black or heavily occluded.

What changed:

- The sensor pose is offset by `0.028 m` along the local camera forward axis.

### 3.4 Add the Gazebo camera sensor tags

Why:

- The sensor must be explicitly enabled and publish both image and camera info.

What changed:

- `always_on`
- `visualize`
- `update_rate`
- `topic`
- `camera_info_topic`
- `gz_frame_id`

### 3.5 Use the official bridge split

Why:

- The official demos use `ros_gz_image` for image topics.
- `CameraInfo` still bridges cleanly with `ros_gz_bridge`.
- This matches the Humble / Fortress examples better than using only `parameter_bridge` for everything.

What changed in `sim.launch.py`:

- Image:
  `ros_gz_image image_bridge /camera/image_raw`
- Camera info:
  `ros_gz_bridge parameter_bridge /camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo`

## 4. Final Camera Sensor Configuration

The validated camera sensor block is in:

- [gazebo.urdf.xacro](/home/xiang/ros2_ws/src/jaka_a5/jaka_a5_description/urdf/gazebo.urdf.xacro)

The validated mount joint is in:

- [jaka_a5.robot.xacro](/home/xiang/ros2_ws/src/jaka_a5/jaka_a5_description/urdf/jaka_a5.robot.xacro)

```xml
<joint name="camera_joint" type="fixed">
  <origin xyz="-0.055 0 0.045" rpy="0 0 3.14159" />
  <parent link="J6" />
  <child link="camera_link" />
</joint>
```

Core structure:

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera_sensor">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>30.0</update_rate>
    <gz_frame_id>camera_link</gz_frame_id>
    <pose>0.028 0 0 0 0 0</pose>
    <topic>/camera/image_raw</topic>
    <camera>
      <camera_info_topic>/camera/camera_info</camera_info_topic>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.01</near>
        <far>100</far>
      </clip>
    </camera>
  </sensor>
</gazebo>
```

## 5. Step-By-Step Validation Procedure

### Step 1. Build the affected packages

```bash
cd /home/xiang/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  jaka_a5_description \
  jaka_a5_gazebo \
  jaka_a5_vision \
  jaka_a5_bringup
source install/setup.bash
```

### Step 2. Launch only the Gazebo camera path

Use the Gazebo-only launch first. Do not start MoveIt or AprilTag while validating the image path.

```bash
cd /home/xiang/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jaka_a5_gazebo sim.launch.py
```

Why:

- This isolates camera problems from controller, MoveIt, and vision stack problems.
- After startup, Gazebo should show a floating `End-Effector Camera` panel inside the GUI.

### Step 3. Check that the ROS topics exist

In a second terminal:

```bash
cd /home/xiang/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic list -t | rg "camera|clock"
```

Expected topics:

- `/camera/image_raw`
- `/camera/camera_info`

### Step 4. Confirm the image publisher is `ros_gz_image`

```bash
ros2 topic info /camera/image_raw --verbose
```

Expected:

- Publisher node name: `ros_gz_image`

This confirms the project is following the official image bridge path.

### Step 5. Confirm camera info is valid

```bash
ros2 topic echo /camera/camera_info --once
```

Expected:

- Non-empty `K`, `P`, `width`, `height`
- `frame_id: camera_link`

### Step 6. Save one image frame to a file

Use this exact command:

```bash
cd /home/xiang/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 - <<'PY'
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

OUT = "/tmp/jaka_camera_frame.png"
bridge = CvBridge()

class Grabber(Node):
    def __init__(self):
        super().__init__("camera_frame_grabber")
        self.done = False
        self.create_subscription(Image, "/camera/image_raw", self.cb, 10)

    def cb(self, msg):
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imwrite(OUT, img)
        print(f"saved={OUT}")
        print(f"shape={img.shape}")
        print(f"mean={float(img.mean()):.3f}")
        print(f"std={float(img.std()):.3f}")
        print(f"min={int(img.min())}")
        print(f"max={int(img.max())}")
        self.done = True

rclpy.init()
node = Grabber()
while rclpy.ok() and not node.done:
    rclpy.spin_once(node, timeout_sec=0.2)
node.destroy_node()
rclpy.shutdown()
PY
```

### Step 7. Interpret the saved image statistics

Validated result on April 20, 2026:

- Shape: `480 x 640 x 3`
- Frequency: about `30 Hz`
- Statistics after the final mount-direction correction:
  - `mean = 101.937`
  - `std = 54.488`
  - `min = 37`
  - `max = 184`

Meaning:

- If `min = max = 0`, the image is fully black.
- If the standard deviation is near zero, the image is almost uniform and likely invalid.
- Here the image has a wide brightness spread and non-uniform content, so the image path is working.

## 6. Why The Previous Black Image Happened

The black or useless image came from a combination of causes:

### Cause 1. The camera model was invisible

- `camera_link` originally had no `visual`.
- Gazebo could simulate the sensor, but the camera body itself was not visible.

### Cause 2. The sensor was effectively looking in the wrong direction

- The wrist frame orientation was not aligned with the workstation.
- The camera initially looked away from the useful workspace.

### Cause 3. The sensor was too close to the camera body

- If the sensor origin sits inside the housing, the housing can occlude the view.
- A small forward offset is a simple and reliable fix.

### Cause 4. The bridge path was not aligned with the official examples

- The official demos separate image bridging and metadata bridging.
- Using `ros_gz_image` for the image topic is the cleanest Humble / Fortress path.

## 7. Current Verified Project State

As of April 20, 2026, the following is verified:

- The end-effector camera model is visible in Gazebo.
- The Gazebo world now contains a dedicated camera display panel.
- Gazebo publishes the camera image topic.
- ROS 2 receives `/camera/image_raw`.
- ROS 2 receives `/camera/camera_info`.
- The image bridge publisher is `ros_gz_image`.
- A frame was saved successfully to `/tmp/jaka_camera_frame_fixed_mount.png`.
- The image stream runs at about `30 Hz`.

## 8. What Is Not Included In This Validation

This document only validates the camera transport path.

It does not guarantee:

- AprilTag detection success
- MoveIt execution success
- Visual servo success

Those belong to the next integration stage after the camera path is stable.

## 9. Recommended Next Step

After camera-only validation succeeds, continue with this order:

1. Validate `apriltag_ros` against the stable `/camera/image_raw` and `/camera/camera_info`.
2. Validate `/tag_detections` output.
3. Validate `/target_pose`.
4. Validate visual servo control.

That sequence prevents camera transport issues from being mixed with higher-level perception and control issues.
