# gimbal_robot_ws

ROS2 workspace for the AI-powered ground gimbal robot internship project.

## Week 2: Mobile Base & Robot Modeling

This package contains a custom differential-drive mobile base for Gazebo
simulation. The model includes:

- wheeled mobile base with left and right drive wheels
- front and rear caster supports for stability
- fixed gimbal mount for the Week 3 camera/gimbal assembly
- Gazebo differential-drive plugin using `/cmd_vel`
- odometry publication on `/odom`
- TF frames for `odom`, `base_footprint`, `base_link`, wheels, mount,
  gimbal, and camera

## Week 3: Pan/Tilt Gimbal & Stabilization Demo

Week 3 extends the mobile base with a simulated pan/tilt camera gimbal. The
new model and control demo include:

- yaw/pan joint mounted above the existing gimbal platform
- pitch/tilt joint and camera body with lens geometry
- `camera_link` and `camera_optical_frame` for future perception work
- Gazebo camera sensor plugin publishing under `/gimbal_camera`
- PID-style gimbal demo node that publishes pan/tilt joint states
- diagnostic topics for simulated target offset and gimbal command output

### Build

```bash
cd ~/gimbal_robot_ws
colcon build
source install/setup.bash
```

### View the model in RViz

```bash
ros2 launch ground_gimbal_robot display.launch.py
```

### Run Week 3 gimbal demo in RViz

```bash
ros2 launch ground_gimbal_robot gimbal_demo.launch.py
```

The demo simulates a moving target offset in the camera image. A small PID
controller converts that offset into pan and tilt joint angles, then publishes
the joints through `/joint_states` so RViz and TF show the gimbal tracking
motion.

### Simulate in Gazebo

```bash
ros2 launch ground_gimbal_robot gazebo.launch.py
```

### Simulate with gimbal demo and camera plugin

```bash
ros2 launch ground_gimbal_robot gazebo_gimbal_demo.launch.py
```

Camera topics are published in the `/gimbal_camera` namespace. For example:

```bash
ros2 topic list | grep gimbal_camera
```

In Gazebo, the gimbal and camera links are held gravity-free so the visual
assembly stays upright without a full Gazebo joint-position controller. The
PID pan/tilt motion is intended for the RViz/TF demo, while Gazebo is used to
verify the physical layout, camera plugin, and mobile robot integration.

### Drive with keyboard teleop

In a second terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Run repeatable motion demo

In a second terminal:

```bash
ros2 launch ground_gimbal_robot motion_demo.launch.py
```

The demo starts with a short pause, drives forward, rotates in place, drives in
an arc, rotates back, and stops. For the arc section, the nominal turning radius
is:

```text
linear velocity / angular velocity = 0.45 / 0.65 = 0.69 m
```

Key model parameters:

```text
Base size:         0.46 m x 0.34 m x 0.12 m
Wheel radius:      0.068 m
Wheel width:       0.045 m
Wheel separation:  0.43 m
Gimbal mount:      0.18 m x 0.14 m x 0.055 m
Pan range:         +/- 170 deg
Tilt range:        +/- 45 deg
Camera body:       0.085 m x 0.055 m x 0.045 m
```

Week 3 diagnostic topics:

```text
/joint_states          gimbal_pan_joint and gimbal_tilt_joint positions
/gimbal/target_offset  simulated image-space target offset
/gimbal/pid_command    commanded pan/tilt joint angles
```

## Week 4: AI Human Detection & Tracking

Week 4 adds a ROS2 perception node that subscribes to the simulated gimbal
camera image, detects a person in the frame, and publishes the normalized
image-space offset for the gimbal controller.

The tracking pipeline includes:

- camera image subscription from `/gimbal_camera/gimbal_camera/image_raw`
- OpenCV color-target detection for the included Gazebo demo subject
- OpenCV HOG-based person detection for real camera/person tests
- optional YOLO detector mode when `ultralytics` and a model file are available
- target center offset calculation relative to the camera frame center
- debug image publication with the detected bounding box
- gimbal controller integration through `/gimbal/target_offset`

### Run Week 4 human tracking demo

```bash
ros2 launch ground_gimbal_robot human_tracking_demo.launch.py
```

The default launch file loads `human_tracking.world`, which places a green
person-shaped target in front of the robot and uses `detector:=color` for a
repeatable simulation demo.

For a safe dynamic recording, run the camera sweep demo:

```bash
ros2 launch ground_gimbal_robot human_tracking_motion_demo.launch.py
```

This version starts the same perception pipeline, then slowly rotates the robot
in place so the camera view and target offset change without driving into the
target.

OpenCV HOG mode for real person detection:

```bash
ros2 launch ground_gimbal_robot human_tracking_demo.launch.py detector:=hog
```

Optional YOLO mode:

```bash
ros2 launch ground_gimbal_robot human_tracking_demo.launch.py \
  detector:=yolo \
  yolo_model:=/path/to/yolov8n.pt
```

Week 4 diagnostic topics:

```text
/gimbal_camera/gimbal_camera/image_raw simulated camera image input
/gimbal/target_offset                 normalized person offset [x, y, confidence]
/gimbal/target_visible                true when a person is currently detected
/gimbal/human_tracking/debug_image    camera image with detection overlay
/gimbal/pid_command                   commanded pan/tilt joint angles
```

The gimbal controller now supports two tracking sources:

```text
tracking_source:=simulated  Week 3 sine-wave target offset demo
tracking_source:=topic      Week 4 vision-driven target offset input
```

## Week 5: Robot Following & Motion Planning

Week 5 adds an autonomous following and local motion-planning demo for the
mobile base. The new nodes reuse the Week 4 camera tracking result, move the
green target through the Gazebo world, and convert the detected image-space
target offset into obstacle-aware `/cmd_vel` commands.

The following pipeline includes:

- target offset input from `/gimbal/target_offset`
- target visibility input from `/gimbal/target_visible`
- odometry input from `/odom`
- a moving green person-shaped target controlled through Gazebo entity state
- static obstacles placed in the Gazebo world
- yaw control that turns the robot until the target is centered in the camera
- forward speed control that approaches the target when it appears far away
- safe-distance behavior that stops or backs up when the target appears close
- local motion-planning logic that slows, stops, and steers away from obstacles
- acceleration limiting so velocity commands change smoothly
- timeout handling so the robot stops when the target is lost

### Run Week 5 person following demo

```bash
ros2 launch ground_gimbal_robot person_following_demo.launch.py
```

The default demo loads the Week 5 human-tracking world with obstacles, starts
the OpenCV color detector, keeps the gimbal centered on the target, starts the
`person_following` local planner, and moves the green target along a repeatable
path. The robot follows the moving subject while maintaining a safe distance
and steering around nearby obstacles.

Useful tuning parameters:

```bash
ros2 launch ground_gimbal_robot person_following_demo.launch.py \
  desired_confidence:=0.60 \
  safe_stop_confidence:=0.84 \
  max_linear_speed:=0.30 \
  max_angular_speed:=0.75
```

Week 5 diagnostic topics:

```text
/cmd_vel                         mobile-base velocity command
/person_following/control_state  [linear, angular, x_error, confidence, visible, avoiding]
/gimbal/target_offset            normalized target offset from Week 4 perception
/gimbal/target_visible           true when the target is currently detected
/odom                            robot pose used by local obstacle planning
```

Motion control analysis:

```text
Angular command:  angular.z = -turn_kp * x_error
Forward command:  linear.x = distance_kp * (desired_confidence - confidence)
Safety stop:      if confidence >= safe_stop_confidence, reverse slowly
Centering rule:   if abs(x_error) is large, rotate first before driving forward
Local planner:    project known obstacles into the robot frame from /odom
Obstacle slow:    reduce linear speed inside the forward safety corridor
Obstacle steer:   add yaw away from the closest obstacle in the corridor
Target timeout:   if no recent target update arrives, smoothly command zero speed
```

## Week 6: 360 Degree Cinematic Tracking

Week 6 adds a cinematic tracking controller on top of the Week 4 perception and
Week 5 motion-control foundation. The robot now supports multi-angle filming
behaviors while the gimbal controller keeps the target framed from the camera
tracking feedback.

The cinematic pipeline includes:

- OpenCV target detection from the simulated gimbal camera
- pan/tilt gimbal tracking from `/gimbal/target_offset`
- a new `cinematic_tracking` mobile-base controller
- orbit, follow-behind, and side-tracking camera modes
- a `showcase` mode that cycles through all three shot types
- smooth linear and angular velocity limiting for camera-friendly motion
- depth-camera distance measurement from the detected target bounding box
- real distance regulation for orbit radius and follow/side tracking distance
- target-loss recovery with slow search motion toward the last seen side
- clean no-obstacle demo world for validating the three cinematic modes first
- diagnostic state output for demo review and tuning

### Run Week 6 cinematic tracking demo

```bash
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py
```

The default `mode:=showcase` runs a multi-angle filming simulation. It starts
with an orbit shot, then switches to side tracking, then follow-behind, and
continues cycling through the modes while the green target moves through the
Gazebo world.

Run one shot type at a time:

```bash
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py mode:=orbit
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py mode:=side_tracking
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py mode:=follow_behind
```

Useful tuning parameters:

```bash
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py \
  mode:=showcase \
  showcase_period:=12.0 \
  orbit_speed:=0.28 \
  orbit_turn_bias:=0.58 \
  side_speed:=0.24 \
  follow_speed:=0.30 \
  desired_distance:=1.80
```

Week 6 diagnostic topics:

```text
/cmd_vel                              mobile-base velocity command
/cinematic_tracking/control_state     [linear, angular, x_error, y_error, confidence, depth_distance_m, visible, avoiding, mode, orbit_direction]
/gimbal/target_offset                 [x_error, y_error, confidence, depth_distance_m]
/gimbal/target_visible                true when the target is currently detected
/gimbal/pid_command                   commanded gimbal pan/tilt angles
/gimbal_position_controller/commands   Gazebo position-controller pan/tilt commands
/gimbal/human_tracking/debug_image    camera image with detection overlay
/odom                                 robot pose used by local obstacle planning
```

Cinematic behavior summary:

```text
Orbit:         drive in a smooth circular path while holding a depth-based radius around the subject
Side tracking: hold a side-biased camera path for lateral-feeling footage
Follow behind: maintain a centered trailing shot with regulated distance
Showcase:      automatically rotate through orbit, side-tracking, and follow-behind shots
Gimbal:        Gazebo pan/tilt joints track the detected target so the camera view moves with the subject
Recovery:      after short target loss, creep and turn toward the last seen side
Avoidance:     slow, steer, or reverse orbit direction near known static obstacles
```

## Week 7: System Integration & Optimization

Week 7 focuses on engineering quality for the complete ground gimbal robot
system. The perception, target filtering, gimbal aiming, cinematic base
controller, moving subject behavior, obstacle-aware local planning, and manual
control safety hooks are integrated into one optimized demo.

The final Week 7 system includes:

- moving green person-shaped target with a longer walking route
- target offset filtering through the `tracking_filter` node
- short tracking-loss prediction with confidence decay
- odom/model-state target tracking for low-latency simulation demos
- real `/cmd_vel` control through the Gazebo differential-drive plugin
- continuous orbit behavior around the moving subject
- target clearance protection so the robot does not collide with the subject
- obstacle-aware orbit expansion around known red/blue static obstacles
- smoother startup, acceleration limiting, and conservative speed caps
- gimbal aiming that keeps the camera pointed at the subject
- manual override support through `/cinematic_tracking/manual_override`

### Run Week 7 optimized system demo

```bash
ros2 launch ground_gimbal_robot week7_system_demo.launch.py
```

The Week 7 launch defaults to `mode:=orbit`. The robot follows the moving target
center, keeps a safe orbit radius, expands the path near obstacles, and keeps the
gimbal aimed at the subject.

Run the multi-shot showcase mode when needed:

```bash
ros2 launch ground_gimbal_robot week7_system_demo.launch.py mode:=showcase
```

Manual override can pause automatic base control so teleoperation or another
`/cmd_vel` publisher can drive the robot:

```bash
ros2 topic pub /cinematic_tracking/manual_override std_msgs/msg/Bool "{data: true}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic pub /cinematic_tracking/manual_override std_msgs/msg/Bool "{data: false}"
```

Week 7 diagnostic topics:

```text
/cmd_vel                              mobile-base velocity command
/odom                                 robot odometry from Gazebo diff-drive
/tracking_subject/odom                moving green subject odometry
/gimbal/target_offset_raw             unfiltered visual target estimate
/gimbal/target_visible_raw            raw detector visibility
/gimbal/target_offset                 filtered/predicted target estimate
/gimbal/target_visible                filtered target visibility
/cinematic_tracking/control_state     integrated controller diagnostics
/cinematic_tracking/manual_override   true pauses automatic base control
/gimbal_position_controller/commands  Gazebo gimbal pan/tilt commands
```

Optimized control behavior:

```text
Startup:        hold zero velocity briefly so Gazebo physics settles
Orbit:          drive tangentially around the moving subject center
Radius control: correct inward/outward drift to maintain filming distance
Target safety:  if too close to the subject, move outward before orbiting
Obstacle logic: expand the orbit radius near known static obstacles
Motion smooth:  apply linear/angular acceleration limits and low speed caps
Tracking loss:  predict briefly, decay confidence, then enter slow recovery
Manual control: pause autonomy without shutting down the ROS2 graph
```

Engineering trade-offs:

```text
Latency vs smoothness:      filtering reduces jitter but can add small delay
Prediction vs safety:       short prediction bridges missed frames but decays quickly
Cinematic path vs clearance: wider orbit improves safety around obstacles
Speed vs stability:         conservative limits prevent Gazebo tipping and overshoot
Integration vs modularity:  separate nodes keep perception, filtering, control, and gimbal logic reusable
Nav2-style vs full Nav2:    simplified local planning is enough for the demo, but not a full Navigation2 stack
```

Demo video link:

```text
[add optimized Week 7 demo video link after recording]
```
