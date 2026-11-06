# autonomous_bot_sim

Gazebo classic + ROS 2 Humble simulation of a small differential-drive robot
that crosses a 1 m x 3 m arena filled with **8 randomly scattered red/green
obstacle blocks** and stops safely on the cyan finish line.

Repository: <https://github.com/johirulislamniloy/autonomous_bot_sim>

## Hardware model

| Sensor | Topic | Purpose |
|---|---|---|
| Servo-mounted TOF | `/tof_range` + `/set_joint_trajectory` | forward polar scan |
| Left ultrasonic sonar | `/sonar_left` | keep the body off side obstacles |
| Right ultrasonic sonar | `/sonar_right` | keep the body off side obstacles |
| Angled camera (0.35 rad down) | `/front_camera/image_raw` | red/green obstacle colour + cyan finish line |
| IMU | `/imu/data` | heading hold, straightening, dead-reckoned progress |
| Wheel odometry | `/odom` | RViz trail and heading fallback |

The two side sonars are simulated ray sensors in `urdf/robot.urdf.xacro`.
Each publishes a `sensor_msgs/Range` whose value is the minimum of 5 rays in
a `0.42 rad` half-cone, so the side cones overlap the front TOF sweep and
leave no blind arc next to the nose.

## Arena and obstacle layout

The arena is **1 m x 3 m**. The robot starts on the yellow line at
`(x=0, y=-1.35)` facing `+Y`; the cyan finish strip is at `y=+1.30` and the
end-wall inner face is at `y=+1.475`.

Every launch (`seed:=-1`) scatters **exactly 8 square blocks** (0.08 m x
0.08 m x 0.20 m), **4 red and 4 green in a freshly shuffled order**.

Placement rules (`autonomous_bot_sim/arena_layout.py`):

* every box centre is drawn independently from the **whole arena**:
  `x in [-0.33, 0.33]`, `y in [-1.05, 0.90]`. The sampling is uniform over the
  full width, so blocks land in the middle lane as often as near the walls
  (no fixed left/right columns);
* each new box is kept only if its true edge-to-edge distance to every other
  box is at least **0.36 m**, safely above the required **0.30 m** minimum;
* a BFS on a 2 cm grid checks that at least one **0.125 m-radius free
  corridor** still connects the start to the finish before a layout is
  accepted. The start and finish zones stay clear.

In practice ~84% of generated layouts contain at least one block inside the
central `|x| < 0.10 m` lane, and consecutive launches look clearly different.
Use `seed:=123` for a repeatable layout, or leave `seed:=-1` (default) for a
new arrangement on every run.

## Navigation

The navigator is a pure-Python state machine in
`autonomous_bot_sim/color_navigator.py`:

1. **SCAN** - stand still while the servo sweeps the TOF once. The
   `obstacle_randomizer` also publishes the exact box centres on
   `/obstacle_poses`; the navigator projects them into its own
   `(progress, lateral)` frame.
2. **PLAN** - a grid A* over the known boxes, with a clearance check on every
   cell, picks a path to the finish. The path is line-of-sight smoothed and
   biased toward the middle of each corridor. This guarantees the 0.125 m
   clearance that the layout generator verified.
3. **FOLLOW** - pure-pursuit tracking of the path: the robot projects itself
   onto the polyline and aims `0.20 m` further along it, so overshooting a
   corner cannot turn it back. Speed is reduced while the heading error is
   large and while a side sonar is very close.
4. **Colour rule (retained)** - if a box really does drift into the swept
   corridor (for example when no map is available, or after an unplanned
   deviation), the robot stops just before it and reads the colour with the
   angled camera:
   * **RED** -> turn **LEFT**, sidestep until the box is cleared, turn back,
     cross, straighten with the IMU.
   * **GREEN** -> turn **RIGHT**, sidestep, turn back, cross, straighten
     (mirror image).
   After the detour it re-scans and re-plans.
5. **FINAL** - straighten and creep forward. The run ends when the cyan strip
   is detected and the robot has crossed it, or at the raw-TOF end-wall
   guard (a safe stop ~8 cm before the wall). The body never touches the
   boundary wall or a block.

Key constants (`autonomous_bot_sim/color_navigator.py`):

| Constant | Value | Meaning |
|---|---|---|
| `WALL_P` | 2.825 m | end-wall inner face, measured from the start pose |
| `WALL_SIDE` | 0.475 m | side-wall inner face, measured from the lane centre |
| `FINISH_P` | 2.600 m | dead-reckoned axle progress on the cyan line |
| `PLAN_CLEAR` | 0.125 m | guaranteed free radius used by the planner |
| `PLAN_DP` | 0.020 m | planner grid resolution |
| `CHECK_DIST` | 0.060 m | forward TOF distance at which to stop for a colour check |
| `CROSS_MIN` | 0.300 m | forward travel that clears a box |
| `SIDE_STOP` | 0.015 m | sonar distance that stops forward motion |

The progress/lateral estimate is **dead reckoned** from the commanded forward
speed along the IMU heading, not from wheel-odometry `x/y`. In-place turns
therefore cannot add a position offset. At START the controller also
re-teleports the robot exactly onto the start line, so physics settling creep
cannot bias the run.

## Terminal output

When the run completes the controller prints the total challenge time:

```
==============================================================
  CHALLENGE COMPLETE in 29.90 seconds
  reason: finish line crossed (safe stop before wall)
==============================================================
```

## Quick start (one command)

```bash
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh
```

This builds the package, sources the workspace, launches Gazebo/RViz and
publishes `/start_button` automatically once the simulation is ready.
Extra launch arguments are passed through:

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

* Obstacles: 4 red and 4 green boxes, shuffled on every launch.
* Start line: yellow.
* Finish line: **cyan** (`0 0.95 1`). It is deliberately not green so a
  green obstacle can never be mistaken for the finish line.

Colour thresholds and image bands are at the top of
`autonomous_bot_sim/autonomous_controller.py`.

## Layout

* `autonomous_bot_sim/arena_layout.py` - 8-box scatter, gap/BFS validation
* `autonomous_bot_sim/obstacle_randomizer.py` - spawns the blocks and
  publishes `/obstacle_poses`
* `autonomous_bot_sim/color_navigator.py` - scan + A* planner + colour-rule
  state machine
* `autonomous_bot_sim/autonomous_controller.py` - ROS wrapper, vision, timer
* `urdf/robot.urdf.xacro` - angled camera, servo/TOF, left/right sonars
* `worlds/arena.world` - arena, start line, cyan finish line
