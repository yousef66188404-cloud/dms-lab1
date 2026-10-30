# Lab 1: ROS 2, Turtlesim, Actions & Services

Design of Mechatronics Systems, Faculty of Engineering, Alexandria National University.

## Team

| Name | ID |
|------|----|
| _Moaz abdelghany abdelhamid_ | _2304225_ |
| _Aly Atef Mohamed _ | _2304150_ |
| _Mohamed Hassan Elhawalka_ | _2304116_ |

## What this repository contains

Three ROS 2 packages (Python nodes, `ament_python`; interfaces and launch files, `ament_cmake`):

| Package | Type | Content |
|---------|------|---------|
| `turtle_interfaces` | `ament_cmake` | `msg/TargetPose` (x, y, yaw), `action/GoToPose`, `action/DrawSquare`, `srv/SetSpeedLimits` |
| `turtle_control` | `ament_python` | `goto_pose_server` (action `go_to_pose` + parameters + service `set_speed_limits`), `reset_service` (service `reset_turtle`), `square_server` (action `draw_square`), unit tests |
| `turtle_bringup` | `ament_cmake` | `system.launch.py`, `square_demo.launch.py`, `config/turtle_params.yaml` |

```
                         ┌────────────────────┐   Twist /turtle1/cmd_vel   ┌──────────────┐
 draw_square (action) ──►│    square_server   │                             │              │
                         │  1. reset_turtle   │──► reset_service ─ /clear ─►│  turtlesim   │
                         │  2. 4 × go_to_pose │      /turtle1/teleport_abs ►│    _node     │
                         └─────────┬──────────┘      /turtle1/set_pen ─────►│              │
                                   ▼                                        │              │
                         ┌────────────────────┐ ◄── Pose /turtle1/pose ─────│              │
 go_to_pose (action) ───►│  goto_pose_server  │ ───────────────────────────►│              │
 set_speed_limits (srv)─►│ params: max speeds │                             └──────────────┘
                         └────────────────────┘
```

Implementation note: the turtle pose arrives at ~60 Hz. Putting it through the `MultiThreadedExecutor` that the
servers need (so a goal can wait for services) made `goto_pose_server` use ~80% CPU and made the velocity commands
jittery, so `turtle_control/pose_tracker.py` listens to the pose in a small helper node (`/goto_pose_listener`)
with its own single-threaded executor (CPU is now ~5%). The control loop runs at a fixed 50 Hz.

### Requirements checklist

| # | Requirement | Where |
|---|-------------|-------|
| R1 | Run turtlesim, control it from our nodes | `system.launch.py`; `goto_pose_server` publishes `/turtle1/cmd_vel` |
| R2 | Custom message x, y, yaw | `turtle_interfaces/msg/TargetPose.msg` |
| R3 | goToPose action server | `turtle_control/goto_pose_server.py`, action `/go_to_pose` |
| R4 | Reset service using turtlesim services | `turtle_control/reset_service.py`, service `/reset_turtle` (uses `/clear`, `/turtle1/set_pen`, `/turtle1/teleport_absolute`) |
| R5 | Square action server, 4 goToPose calls | `turtle_control/square_server.py`, action `/draw_square` |
| R6 | Everything through launch files | `turtle_bringup/launch/` |
| R7 | Speed parameters + service to change them | parameters `max_linear_speed`, `max_angular_speed` of `/goto_pose_server`; service `/set_speed_limits` |
| R8 | Well-structured packages | the three packages above |
| Bonus | Design patterns, bag file | sections below |

Extras beyond the requirements: goals outside the window or while the turtle is busy are rejected, goals can be
canceled, speed parameters are validated (≤ 0, NaN, wrong type are refused) by one callback used by both
`ros2 param set` and our service, tolerances are parameters, the code works on Humble and on Jazzy
(turtlesim message package was renamed), the motion logic has unit tests.

## Setup (Ubuntu 22.04 with Humble, or Ubuntu 24.04 with Jazzy)

On Windows 11 use WSL2 (turtlesim opens its window through WSLg): in PowerShell run
`wsl --install -d Ubuntu-24.04`, reboot, open Ubuntu, then follow the Ubuntu steps below.

```bash
# ROS 2 Jazzy (for Humble replace "jazzy" with "humble"); see docs.ros.org for the full install guide
sudo apt update && sudo apt install -y software-properties-common curl git
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt install -y ros-jazzy-desktop ros-jazzy-turtlesim ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## Build

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <THIS_REPOSITORY_URL> dms_lab1
cd ~/ros2_ws
sudo rosdep init 2>/dev/null; rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash          # in EVERY new terminal
```

## Run

```bash
ros2 launch turtle_bringup system.launch.py          # turtlesim + all servers + parameters
ros2 launch turtle_bringup square_demo.launch.py     # same, then draws a square automatically
ros2 launch turtle_bringup square_demo.launch.py side_length:=4.0 record_bag:=true   # + bag file
```

## Try every requirement (start `system.launch.py` first, one command per terminal)

**R1 - turtlesim and topics/services**
```bash
ros2 run turtlesim turtle_teleop_key        # arrow keys move the turtle
ros2 topic list -t                          # /turtle1/pose, /turtle1/cmd_vel
ros2 service list -t                        # /clear, /reset, /spawn, /turtle1/teleport_absolute ...
ros2 node list                              # /turtlesim /goto_pose_server /reset_service /square_server (+ /goto_pose_listener, see below)
```

**R2 - interfaces**
```bash
ros2 interface list | grep turtle_interfaces
ros2 interface show turtle_interfaces/msg/TargetPose
ros2 interface show turtle_interfaces/action/GoToPose
ros2 interface show turtle_interfaces/action/DrawSquare
ros2 interface show turtle_interfaces/srv/SetSpeedLimits
```

**R7 - parameters and speed service**
```bash
ros2 param list /goto_pose_server
ros2 param get /goto_pose_server max_linear_speed
ros2 param set /goto_pose_server max_linear_speed 1.0
ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: 1.5, max_angular_speed: 1.0}"
ros2 param get /goto_pose_server max_angular_speed          # 1.0
ros2 service call /set_speed_limits turtle_interfaces/srv/SetSpeedLimits "{max_linear_speed: -1.0, max_angular_speed: 1.0}"   # success: false
ros2 param set /goto_pose_server max_linear_speed 0.0       # refused
```

**R3 - goToPose** (change the speeds while it moves to see the effect live)
```bash
ros2 action list -t
ros2 action send_goal --feedback /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 8.0, y: 3.0, yaw: 1.57}}"
ros2 topic echo /turtle1/pose --once                         # compare with the goal
ros2 action send_goal --feedback /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 2.0, y: 9.0, yaw: -2.0}}"
ros2 action send_goal /go_to_pose turtle_interfaces/action/GoToPose "{target: {x: 15.0, y: 3.0, yaw: 0.0}}"   # rejected: outside the window
```

**R4 - reset**
```bash
ros2 service call /reset_turtle std_srvs/srv/Trigger
ros2 topic echo /turtle1/pose --once                         # x = y = 5.544445, theta = 0
```

**R5 - square** (the `goto_pose_server` log shows `Goal #1 ... Goal #4`)
```bash
ros2 action send_goal --feedback /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}"
ros2 action send_goal --feedback /draw_square turtle_interfaces/action/DrawSquare "{side_length: 0.0}"   # default side 3.0, second run starts clean
```

**R6 - launch files**: close all terminals, open one, `source ~/ros2_ws/install/setup.bash`, run only
`ros2 launch turtle_bringup system.launch.py`, then `ros2 node list` and `ros2 param get /goto_pose_server max_linear_speed`.
The values come from `turtle_bringup/config/turtle_params.yaml` (use another file with `params_file:=/path/file.yaml`).

## Unit tests

The motion logic is plain Python and is tested against a simulation of turtlesim's kinematics
(all directions, all yaws, window corners, goal behind the turtle, live speed change, square geometry):

```bash
colcon test --packages-select turtle_control && colcon test-result --verbose
# or, without ROS:  cd turtle_control && python3 -m pytest test
```

## Bonus 1: design patterns

| Pattern | Where | Why |
|---------|-------|-----|
| **State** | `turtle_control/motion.py`: `RotateToTarget` → `DriveToTarget` → `RotateToYaw`, run by `GoToPoseMotion` | A goToPose motion has three phases with different behaviour. Each phase is a class that knows its command and its next state, so the action server only calls `motion.update(pose, limits)` and has no `if phase == ...` chains. Adding a phase (for example "back up") does not touch the others. |
| **Strategy** | `Controller` / `ProportionalController` in `motion.py` | The way speeds are computed from errors is separate from the phases. A different controller (PID, trapezoidal profile) can be injected into `GoToPoseMotion` without changing any state. It also applies the live speed limits. |
| **Factory Method** | `WaypointFactory.create(shape, ...)` in `turtle_control/waypoints.py` | `square_server` asks the factory for the list of goals and does not know how corners are computed. A new shape is one function and one registry entry. |

`motion.py` and `waypoints.py` have no ROS imports, which is also what makes them unit-testable. The ROS
nodes (`goto_pose_server.py`, `square_server.py`, `reset_service.py`) only do communication.

## Bonus 2: bag file

```bash
# terminal 1
ros2 launch turtle_bringup system.launch.py
# terminal 2: record, then draw the square (or use  square_demo.launch.py record_bag:=true)
ros2 bag record -o square_bag /turtle1/cmd_vel /turtle1/pose
ros2 action send_goal --feedback /draw_square turtle_interfaces/action/DrawSquare "{side_length: 3.0}"
# Ctrl+C in the recording terminal, then:
ros2 bag info square_bag                                      # topics and message counts > 0
ros2 service call /reset_turtle std_srvs/srv/Trigger          # same start pose as the recording
ros2 bag play square_bag --topics /turtle1/cmd_vel            # the turtle repeats the square
```
We replay only `/turtle1/cmd_vel`: replaying `/turtle1/pose` too would fight with the pose that turtlesim itself publishes.

The bag of our own test run is in `turtle_bringup/bags/square_bag` (772 velocity commands, 2010 poses, ~32 s).
Replaying it from a reset turtle ended at x=5.52, y=5.61, theta=-0.03, i.e. the square closed back at the start
(start pose: 5.544, 5.544, 0.0).

_Add a screenshot of the replay here._

## Troubleshooting

| Problem | Fix |
|---------|-----|
| `Package 'turtle_...' not found` | `source ~/ros2_ws/install/setup.bash` in this terminal; rebuild. |
| Goal rejected | Target must be inside 0..11, and one goal at a time. Check `ros2 action list`. |
| `reset_turtle` fails | `turtlesim_node` is not running. |
| Changes in the YAML have no effect | With `--symlink-install` relaunch; otherwise rebuild first. |
