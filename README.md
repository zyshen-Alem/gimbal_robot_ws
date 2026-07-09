# AI-Powered Ground Gimbal Robot

ROS 2 Humble workspace for an internship robotics project focused on visual person tracking, mobile filming, gimbal control, and local obstacle avoidance in Gazebo.

The final system uses a simulated differential-drive robot with a pan/tilt camera, YOLO-based human detection, depth-assisted distance estimation, LiDAR obstacle sensing, and runtime-selectable filming modes. The project was developed across weekly milestones, and the final Week 8 demo integrates the earlier robot model, perception, gimbal, following control, and cinematic tracking work into one Gazebo scenario.

## Project Goals

The goal is to simulate a ground robot that can film a walking person in a more cinematic way than a fixed forward-facing follower. The robot should:

- detect and track a person from the onboard gimbal camera;
- keep the camera/gimbal aimed at the person;
- follow the person while maintaining a safe filming distance;
- avoid nearby obstacles with local LiDAR-based behavior;
- support runtime switching between follow, side tracking, and orbit filming;
- demonstrate the final behavior in repeatable Gazebo scenarios.

## Final Capabilities

- **YOLO visual tracking**: the `human_tracking` node detects the actor from the gimbal camera image and publishes target offset, visibility, confidence, and depth-assisted distance.
- **Pan/tilt gimbal aiming**: the `gimbal_demo` node controls the simulated pan/tilt gimbal and keeps the camera aimed toward the person during cinematic modes.
- **Follow mode**: the base follows the walking subject while regulating distance and centering the subject in view.
- **Side tracking mode**: the robot can run beside the subject for lateral filming.
- **Orbit CW/CCW mode**: the robot can orbit around the subject clockwise or counterclockwise while the gimbal points at the subject.
- **Local obstacle avoidance**: the base uses LiDAR scan sectors to detect obstacles and execute local avoid-turn, edge-follow, and rejoin behavior.
- **Runtime mode switching**: commands are sent through `/tracking_mode_cmd`, so one launch can demonstrate multiple filming modes without restarting the simulation.

## Final Demo: YOLO Actor Following

This is the main Week 8 integration demo and the recommended entry point for final review.

```bash
cd ~/gimbal_robot_ws
colcon build
source install/setup.bash
ros2 launch ground_gimbal_robot yolo_actor_following_demo.launch.py
```

The launch file starts the full final pipeline:

- Gazebo world: `worlds/yolo_actor_following.world`
- Robot model: `urdf/ground_gimbal_robot.xacro`
- YOLO detector: `human_tracking`
- Gimbal controller: `gimbal_demo`
- Mobile base controller: `person_following`
- Runtime mode helper: `tracking_mode_director`
- Gazebo gimbal joint controllers through `ros2_control`

The default mode is `follow`. Runtime mode changes are sent through `/tracking_mode_cmd`.

Use repeated publishing instead of `--once` when possible. A one-shot ROS 2 publisher can occasionally exit before discovery completes, so the command may need to be sent twice if `--once` is used.

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: orbit_cw}" -r 2 --times 3
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: orbit_ccw}" -r 2 --times 3
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: side_tracking}" -r 2 --times 3
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: follow}" -r 2 --times 3
```

Useful launch parameters:

```bash
ros2 launch ground_gimbal_robot yolo_actor_following_demo.launch.py \
  orbit_speed:=0.19 \
  orbit_radius:=2.0 \
  side_distance:=2.0 \
  yolo_model:=yolov8n.pt
```

## Recommended Final Demo Scenarios

The final videos can be recorded as two repeatable scenarios.

### Demo 1: Follow to Orbit CW

1. Start the final demo launch.
2. Let the actor walk between the red and blue obstacles.
3. Let the robot follow the actor using YOLO and LiDAR local avoidance.
4. After the actor reaches the green obstacle area and stops, publish:

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: orbit_cw}" -r 2 --times 3
```

This demonstrates walking subject tracking, obstacle-aware following, and clockwise orbit filming around the stopped subject.

### Demo 2: Side Tracking to Orbit CCW

1. Start the final demo launch.
2. Publish side tracking:

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: side_tracking}" -r 2 --times 3
```

3. After the actor reaches the green obstacle area and stops, publish:

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: orbit_ccw}" -r 2 --times 3
```

This demonstrates side tracking, runtime mode switching, and counterclockwise orbit filming.

Together, the two videos cover the Week 8 demo requirements: walking subject tracking, orbit filming, and indoor-style navigation around obstacles.

## System Architecture

```text
Gazebo world
  -> RGB camera image
  -> depth camera image
  -> LiDAR scan
  -> robot odometry

RGB + depth camera
  -> human_tracking node (YOLO)
  -> /gimbal/target_offset
  -> /gimbal/target_visible

/gimbal/target_offset
  -> gimbal_demo node
  -> pan/tilt commands
  -> /gimbal_position_controller/commands

/gimbal/target_offset + /gimbal/pid_command + /scan + /odom
  -> person_following node
  -> follow / side / orbit mode controller
  -> local obstacle avoidance
  -> /cmd_vel

/cmd_vel
  -> Gazebo diff-drive plugin
  -> robot base motion

/tracking_mode_cmd
  -> person_following: switches base behavior
  -> gimbal_demo: switches follow vs cinematic gimbal aiming
  -> tracking_mode_director: controls scripted actor continuation for side tracking demo
```

## Runtime Mode Logic

### Follow Mode

Follow mode is the default mode. The robot uses the visual target offset and distance estimate to keep the person centered and maintain a safe trailing distance. In this mode, the base and gimbal are mostly aligned for a conventional following shot.

Primary inputs:

- `/gimbal/target_offset`
- `/gimbal/target_visible`
- `/gimbal/pid_command`
- `/scan`
- `/odom`

Primary output:

- `/cmd_vel`

### Side Tracking Mode

Side tracking moves the robot toward a side-biased position relative to the person, producing a lateral filming effect. In the final demo, the `tracking_mode_director` helps continue the actor motion when side tracking is requested so the side-tracking behavior can be shown after the initial follow sequence.

Command:

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: side_tracking}" -r 2 --times 3
```

### Orbit Mode

Orbit mode locks the current person position as the orbit center and drives the robot around that center. The gimbal remains aimed at the person while the base follows a circular path.

Clockwise:

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: orbit_cw}" -r 2 --times 3
```

Counterclockwise:

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: orbit_ccw}" -r 2 --times 3
```

The final implementation prioritizes safe obstacle clearance during orbit mode. Around the green obstacle, the local planner may choose a wider route to avoid collision before rejoining the orbit path.

## Local Obstacle Avoidance

The final demo uses a reactive LiDAR local planner in `person_following.py`. It does not use a full global planner; instead, it monitors scan sectors around the robot and modifies the commanded base velocity when obstacles are close.

The scan is divided into:

- front sector: detects obstacles directly ahead;
- left side sector: estimates clearance on the left;
- right side sector: estimates clearance on the right.

The avoidance state machine is:

```text
follow_goal
  -> normal tracking command is used
  -> if the front sector is blocked, start avoidance

avoid_turn
  -> turn toward the safer side for a short time
  -> creates initial clearance from the obstacle

avoid_edge
  -> follow the obstacle edge while maintaining a target side distance
  -> continue until the front and side sectors are safe

rejoin_goal
  -> return to the active filming mode command
  -> if the path becomes blocked again, go back to avoid_edge
```

Important tuning parameters in `yolo_actor_following_demo.launch.py`:

```text
scan_clear_distance             distance considered clear for rejoining
avoidance_trigger_distance      front distance that starts avoidance
scan_stop_distance              emergency close-range threshold
avoid_edge_distance             desired side distance while edge-following
avoid_arc_speed                 forward speed during avoidance
avoid_turn_speed                turn speed during the first avoid-turn phase
avoid_arc_turn_speed            base turn bias during edge following
avoid_arc_min_duration          minimum time before rejoining the goal
```

## Gimbal and Camera Control

The robot uses a pan/tilt gimbal mounted on the base. In follow mode, the gimbal can return toward the front-facing direction so the base and camera look aligned. In orbit and side-tracking modes, the gimbal is decoupled from the base heading so the camera can remain aimed at the subject while the robot moves sideways or around the person.

Important gimbal parameters:

```text
target_height      vertical point on the actor to aim at; lower values include more legs/feet
pan_rate_limit     maximum pan motion speed
tilt_rate_limit    maximum tilt motion speed
search_pan_limit   search sweep range when the visual target is lost
search_period      search sweep period
```

If the camera view is too high and misses the feet, reduce `target_height` in `yolo_actor_following_demo.launch.py`. For example:

```text
target_height: 0.45  -> upper body framing
target_height: 0.30  -> lower framing with more legs/feet
```

## Main Files

| Path | Description |
| --- | --- |
| `README.md` | Project overview and final demo instructions. |
| `src/ground_gimbal_robot/launch/yolo_actor_following_demo.launch.py` | Main Week 8 final demo launch. |
| `src/ground_gimbal_robot/worlds/yolo_actor_following.world` | Final actor-and-obstacle Gazebo world. |
| `src/ground_gimbal_robot/urdf/ground_gimbal_robot.xacro` | Robot model, gimbal, camera, LiDAR, diff-drive, and Gazebo plugins. |
| `src/ground_gimbal_robot/ground_gimbal_robot/human_tracking.py` | Detection node supporting color, HOG, and YOLO modes. |
| `src/ground_gimbal_robot/ground_gimbal_robot/gimbal_demo.py` | Pan/tilt gimbal controller. |
| `src/ground_gimbal_robot/ground_gimbal_robot/person_following.py` | Final follow, side, orbit, and LiDAR local-planning controller. |
| `src/ground_gimbal_robot/ground_gimbal_robot/tracking_mode_director.py` | Runtime helper for final demo mode commands and actor continuation. |
| `src/ground_gimbal_robot/config/gimbal_controllers.yaml` | ROS 2 control configuration for gimbal joints. |

## Core Nodes

| Node | File | Purpose |
| --- | --- | --- |
| `human_tracking` | `ground_gimbal_robot/human_tracking.py` | Detects the person using color, HOG, or YOLO and publishes image-space target offset. |
| `gimbal_demo` | `ground_gimbal_robot/gimbal_demo.py` | Controls pan/tilt gimbal position from target offset or simulation target pose. |
| `person_following` | `ground_gimbal_robot/person_following.py` | Converts visual tracking, odometry, and LiDAR into `/cmd_vel` for follow, side, and orbit modes. |
| `tracking_mode_director` | `ground_gimbal_robot/tracking_mode_director.py` | Supports runtime mode commands and scripted actor continuation for the final demo. |
| `cinematic_tracking` | `ground_gimbal_robot/cinematic_tracking.py` | Earlier cinematic controller used for Week 6/7 milestone demos. |
| `tracking_filter` | `ground_gimbal_robot/tracking_filter.py` | Earlier target filtering and prediction helper used by Week 7 demos. |
| `moving_target_demo` | `ground_gimbal_robot/moving_target_demo.py` | Earlier scripted target motion helper for pre-YOLO demos. |

## Important Topics

```text
/gimbal_camera/gimbal_camera/image_raw              RGB image input for YOLO
/gimbal_camera/gimbal_depth_camera/depth/image_raw  depth input for distance estimation
/gimbal/target_offset                               [x_error, y_error, confidence, distance]
/gimbal/target_visible                              detector visibility flag
/gimbal/pid_command                                 current gimbal pan/tilt state
/gimbal_position_controller/commands                Gazebo gimbal position commands
/scan                                               LiDAR scan for local obstacle avoidance
/odom                                               robot odometry
/cmd_vel                                            mobile-base velocity command
/person_following/control_state                     controller diagnostics
/tracking_mode_cmd                                  runtime filming mode command
```

## Repository Layout

```text
src/ground_gimbal_robot/
  config/       RViz, ros2_control, and navigation configuration
  launch/       Gazebo, milestone, and final demo launch files
  urdf/         ground robot, gimbal, camera, LiDAR, and Gazebo plugins
  worlds/       Gazebo worlds for historical and final demos
  ground_gimbal_robot/
                Python ROS 2 nodes
```

## Historical Milestone Demos

The repo keeps earlier weekly demos because they document project progress and provide smaller tests for individual subsystems.

### Week 2-3: Robot Model and Gimbal

```bash
ros2 launch ground_gimbal_robot display.launch.py
ros2 launch ground_gimbal_robot gimbal_demo.launch.py
ros2 launch ground_gimbal_robot gazebo_gimbal_demo.launch.py
ros2 launch ground_gimbal_robot motion_demo.launch.py
```

These demos validate the robot geometry, differential drive base, gimbal model, camera plugin, and simple motion commands.

### Week 4: Camera-Based Human Tracking

```bash
ros2 launch ground_gimbal_robot human_tracking_demo.launch.py
ros2 launch ground_gimbal_robot human_tracking_motion_demo.launch.py
ros2 launch ground_gimbal_robot human_tracking_demo.launch.py detector:=yolo yolo_model:=yolov8n.pt
```

These demos validate camera input, target detection, debug image output, and target offset publishing.

### Week 5: Person Following and Local Planning

```bash
ros2 launch ground_gimbal_robot person_following_demo.launch.py
```

This demo validates early person-following behavior with local obstacle-aware motion control.

### Week 6: Cinematic Tracking Modes

```bash
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py mode:=orbit
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py mode:=side_tracking
ros2 launch ground_gimbal_robot cinematic_tracking_demo.launch.py mode:=follow_behind
```

This demo validates earlier orbit, side-tracking, and follow-behind behaviors before the final YOLO actor workflow.

### Week 7: Integrated Optimization Demo

```bash
ros2 launch ground_gimbal_robot week7_system_demo.launch.py
ros2 launch ground_gimbal_robot week7_system_demo.launch.py mode:=showcase
```

This demo preserves the Week 7 integration work, including filtered target estimates and the earlier cinematic controller.

## Optional / Experimental Files

Some files are kept as experiment branches or milestone tests rather than final-demo dependencies.

| File | Status |
| --- | --- |
| `launch/yolo_actor_tracking_demo.launch.py` | Useful YOLO actor perception-only test. Not required for final demo. |
| `worlds/yolo_actor_test.world` | Small world for YOLO actor tracking tests. Not required for final demo. |
| `launch/nav2_yolo_following_demo.launch.py` | Experimental Nav2-style YOLO following launch. Not part of the final Week 8 video workflow. |
| `worlds/nav2_yolo_following.world` | World for the experimental Nav2-style branch. |
| `config/nav2_yolo_following.yaml` | Navigation parameters for the experimental Nav2-style branch. |
| `ground_gimbal_robot/nav2_yolo_goal.py` | Experimental helper for sending Nav2 goals from YOLO tracking. |

These files are intentionally not deleted because they document exploration paths and may be useful if the project is extended toward full Navigation2.

## Troubleshooting

### Mode command does not work the first time

Use repeated publishing:

```bash
ros2 topic pub /tracking_mode_cmd std_msgs/String "{data: orbit_cw}" -r 2 --times 3
```

A single `--once` command can publish before subscriber discovery is complete.

### `rqt_image_view` is not found

On ROS 2 Humble, the package may be installed but the executable may not be on the current shell path until ROS is sourced correctly.

```bash
source /opt/ros/humble/setup.bash
cd ~/gimbal_robot_ws
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Then select the image topic, usually:

```text
/gimbal_camera/gimbal_camera/image_raw
```

### Camera looks too high

Lower the `target_height` parameter in `yolo_actor_following_demo.launch.py`.

### Orbit is too slow

Increase `orbit_speed` at launch time:

```bash
ros2 launch ground_gimbal_robot yolo_actor_following_demo.launch.py orbit_speed:=0.20
```

If the robot does not get faster, check `max_linear_speed` in the launch file because it caps the commanded speed.

### Orbit avoids obstacles but takes a wide path

The final orbit avoidance tuning prioritizes avoiding collisions. The local planner may choose a wider path around the green obstacle before rejoining the orbit trajectory.

## Current Limitations

- The final demo is a Gazebo simulation, not a hardware deployment.
- The local planner is reactive and LiDAR-based; it is not a full global Navigation2 stack.
- The final scenario is tuned for repeatable internship demo videos rather than arbitrary environments.
- Orbit CW and CCW can behave differently around asymmetric obstacle layouts because the orbit tangent direction interacts with local avoidance geometry.
- Some historical demos use color targets, odometry/model-state shortcuts, or earlier controllers; the final Week 8 demo uses the YOLO actor workflow.
- The experimental Nav2 files are kept for future work but are not the main final demo path.

## Development Environment

Tested with:

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic with `gazebo_ros`
- Python ROS 2 nodes installed through `colcon build`
- Optional YOLO detector through `ultralytics` using a model such as `yolov8n.pt`

## Build

```bash
cd ~/gimbal_robot_ws
colcon build
source install/setup.bash
```

## Final Deliverables Supported by This Repo

This repository supports the Week 8 final deliverables:

- final integrated Gazebo demo;
- demo videos showing follow, side tracking, orbit, gimbal aiming, and obstacle avoidance;
- technical report system diagrams and architecture explanation;
- final presentation deck with demo scenarios and engineering trade-offs.