# autonomous_bot_sim

Gazebo classic + ROS 2 Humble simulation of a small differential-drive robot
that finds its way through a 1 m x 3 m arena filled with **10 coloured
obstacles** and stops on the finish line.

## Hardware model

| Sensor | Topic | Purpose |
|---|---|---|
| Servo-mounted TOF | `/tof_range` + `/set_joint_trajectory` | 180-degree polar scan |
| Left ultrasonic sonar | `/sonar_left` | keep the body off side obstacles |
| Right ultrasonic sonar | `/sonar_right` | keep the body off side obstacles |
| Angled camera (0.35 rad down) | `/front_camera/image_raw` | red/green obstacle colour + cyan finish line |
| IMU | `/imu/data` | heading hold, straightening, dead-reckoned progress |
| Wheel odometry | `/odom` | RViz trail and heading fallback |

The two side sonars are real simulated ray sensors in
`urdf/robot.urdf.xacro`; each publishes a `sensor_msgs/Range` whose value is
the minimum of 5 rays in a 30-degree cone (`radiation_type = ultrasound`).

## Arena and gate layout

The arena is **1 m x 3 m**. The robot starts on the yellow line at
`(x=0, y=-1.35)` facing `+Y`; the cyan finish strip is at `y=+1.30` and the
end-wall inner face is at `y=+1.475`.

The ten boxes form **five compact colour gates** at rows
`y = -1.10, -0.53, 0.04, 0.61, 1.18` (~0.57 m apart). Each gate has one box
almost on the centre line and a second box only `0.15 m` to the closed side,
so either colour rule always leaves a wide lane:

* **RED gate** -> pair on the robot's right -> turn **LEFT**, sidestep to
  `lat = +0.22 m`, cross on the left.
* **GREEN gate** -> pair on the robot's left -> turn **RIGHT**, sidestep to
  `lat = -0.22 m`, cross on the right.

Gate colours alternate (R, G, R, G, R with seed 123), giving six red and four
green boxes; the first colour is random for other seeds.

Key navigator constants (`autonomous_bot_sim/color_navigator.py`):

| Constant | Value | Meaning |
|---|---|---|
| `GATES` | 5 | colour gates (pairs) to pass |
| `WALL_P` | 2.825 m | end-wall inner face, measured from the start pose |
| `WALL_SIDE` | 0.475 m | side-wall inner face, measured from the lane centre |
| `FINISH_P` | 2.620 m | axle progress on the cyan finish line |
| `PASS_LAT` | 0.220 m | absolute lateral target for a sidestep |
| `CHECK_DIST` | 0.060 m | forward TOF distance at which to stop for the colour check |
| `CROSS_MIN` | 0.300 m | forward travel that clears a gate |
| `RECENTER_DIST` | 0.180 m | forward distance allowed to re-centre |

## Challenge sequence

1. **SCAN** - stand still while the servo rotates the TOF once
   (left -> right). The polar map is used to choose the longest free path
   toward the finishing wall.
2. **APPROACH** - drive at the gate box. Far away the TOF bearing steers;
   close up the camera colour centroid / pure-pursuit lane hold centres the
   box. The side sonars steer away from anything that would touch the body.
3. **CHECK** - stop just before the box and sample the angled camera for
   0.55 s (lower image rows only, so the next gate cannot win the vote).
4. **RED gate** - turn LEFT, sidestep to the absolute left pass lane, turn
   back, cross the gate, then re-centre.
5. **GREEN gate** - turn RIGHT, sidestep to the absolute right pass lane,
   turn back, cross the gate, then re-centre.
6. Back to step 2 for the next gate (no second full scan). This repeats five
   times.
7. **FINAL** - straighten and creep forward; stop as soon as the cyan finish
   strip is detected. A raw TOF end-wall guard (`/tof_range`) is always
   active, so the body never touches the boundary wall even if the colour is
   not visible.

The navigator's progress/lateral estimate is **dead reckoned** from the
commanded forward speed along the IMU heading, not from wheel-odometry `x/y`.
In-place turns therefore cannot add a position offset, which used to make the
robot stop short of the last gate or the finish line. At START the controller
also re-teleports the robot exactly onto the start line, so physics settling
creep cannot bias the run.

When the run completes the controller prints the total time in the terminal:

```
==============================================================
  CHALLENGE COMPLETE in 103.30 seconds
  reason: finish line colour detected
==============================================================
```

## Quick start (one command)

```bash
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh
```

This builds the package, sources the workspace, launches Gazebo/RViz and
publishes `/start_button` automatically after the simulation is ready.
Extra launch arguments are passed through, e.g.:

```bash
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh gui:=false rviz:=false seed:=123
```

A `botsim` alias is also available in new interactive shells:

```bash
botsim
```

To keep pressing START manually, add `auto_start:=false`:

```bash
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh auto_start:=false
```

## Run

```bash
cd ~/ros2_ws
colcon build --packages-select autonomous_bot_sim --symlink-install
source install/setup.bash
ros2 launch autonomous_bot_sim sim.launch.py          # add gui:=false for headless
```

In a second terminal press ENTER to release the start signal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_bot_sim start_button
```

Useful launch options:

```bash
ros2 launch autonomous_bot_sim sim.launch.py seed:=123      # repeatable layout
ros2 launch autonomous_bot_sim sim.launch.py rviz:=false    # no RViz
ros2 launch autonomous_bot_sim sim.launch.py gui:=false     # no Gazebo GUI
```

## Colours

* Obstacles: ten boxes arranged as five same-colour gates; consecutive gates
  alternate colour (6 red / 4 green, or 4 red / 6 green if green comes first).
* Start line: yellow.
* Finish line: **cyan** (`0 0.95 1`). It is deliberately not green so a
  green obstacle can never be mistaken for the finish line.

Colour thresholds and image bands are at the top of
`autonomous_bot_sim/autonomous_controller.py`.

## Layout

* `autonomous_bot_sim/arena_layout.py` - obstacle count, placement, colours
* `autonomous_bot_sim/obstacle_randomizer.py` - spawns the coloured obstacles
* `autonomous_bot_sim/color_navigator.py` - pure-Python course state machine
* `autonomous_bot_sim/autonomous_controller.py` - ROS wrapper + vision + timer
* `urdf/robot.urdf.xacro` - angled camera, servo/TOF, left/right sonars
* `worlds/arena.world` - arena, start line, cyan finish line
