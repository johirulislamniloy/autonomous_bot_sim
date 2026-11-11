# autonomous_bot_sim

Gazebo classic + ROS 2 Humble simulation of a small differential-drive robot
for the **two rounds of the autonomous bot competition**.

| Round | World | Goal |
|---|---|---|
| **Obstacle round** (default) | `worlds/obstacle_round.world` | cross a 1 m x 3 m arena filled with 8 scattered blocks and stop on the finish line |
| **Open round** | `worlds/open_round.world` | follow a 1 m wide square ring for 3 complete laps and stop on the starting side |

Both rounds use the same robot, the same 5 s stationary start, and the same
`run_sim.sh` one-command workflow. A 10 cm wall is used in both worlds.

Repository: <https://github.com/johirulislamniloy/autonomous_bot_sim>

## Hardware model

| Sensor | Topic | Purpose |
|---|---|---|
| Servo-mounted TOF | `/tof_range` + `/set_joint_trajectory` | forward polar scan / corner detection |
| Left ultrasonic sonar | `/sonar_left` | track following and side clearance |
| Right ultrasonic sonar | `/sonar_right` | track following and side clearance |
| Angled camera (0.35 rad down) | `/front_camera/image_raw` | cyan finish strip (obstacle round) |
| IMU | `/imu/data` | heading hold, straightening, 90 deg corner turns |
| Wheel odometry | `/odom` | RViz trail and heading fallback |

The two side sonars are simulated ray sensors in `urdf/robot.urdf.xacro`.
Each publishes a `sensor_msgs/Range` whose value is the minimum of 5 rays in
a `0.30 rad` half-cone, so the side cones overlap the front TOF sweep. The
servo/TOF pivot sits at ~8 cm above the floor, below the top of the 10 cm
walls, so it sees both walls and blocks.

The simulated ray length (`ray_max`) is 5.5 m, longer than either world's
diagonal. `libgazebo_ros_ray_sensor` reports `FLT_MAX` when a ray hits nothing,
and rclpy cannot deserialise that value; with a long-enough ray every
horizontal ray in an enclosed world hits a wall, so only finite ranges are
published.

---

## Obstacle round (default)

The arena is **1 m x 3 m** with **10 cm tall** boundary walls. The robot
starts on the yellow line at `(x=0, y=-1.35)` facing `+Y`; the cyan finish
strip is at `y=+1.175`, 30 cm before the end-wall inner face.

Every launch (`seed:=-1`) scatters **exactly 8 black-edged competition blocks**
of **5 cm x 5 cm x 10 cm**, 4 red and 4 green in a freshly shuffled order.
Colours are visual only; the navigator never reads them.

Placement rules (`autonomous_bot_sim/arena_layout.py`):

* every box centre is drawn from the whole arena
  `x in [-0.33, 0.33]`, `y in [-1.05, 0.90]`;
* each new box keeps a true edge-to-edge distance of at least **0.42 m** to
  every other box (above the required 0.30 m minimum), so the reactive modes
  keep a full robot-width corridor;
* a BFS on a 2 cm grid checks that a 0.125 m-radius free corridor still
  connects the start to the finish before a layout is accepted.

The run starts with a **5 s stationary hold**, counted in the challenge time.
The obstacle-round modes are `astar`, `scan` and `bug2`.

```bash
./run_sim.sh                  # astar (default)
./run_sim.sh mode:=scan
./run_sim.sh mode:=bug2
./run_sim.sh seed:=123        # repeatable layout
```

### Obstacle-round navigation (`mode:=astar|scan|bug2`)

* **`astar`** - the randomizer publishes the exact box centres on
  `/obstacle_poses`; a grid A* with a 0.125 m clearance finds a path to the
  finish, which is followed with pure pursuit. If no path exists it falls back
  to the reactive scan behaviour and replans later.
* **`scan`** - reactive scan-and-go: stand still and sweep the servo/TOF, pick
  the most open bearing, drive straight, then pass each box on the side that
  bends back toward the arena middle.
* **`bug2`** - classic Bug2 on the finish line: drive along the M-line, and on
  a hit follow the box boundary until the line is clear again.
* **FINAL** (all three) - straighten, ease back to the lane centre and creep
  onto the cyan strip. The robot stops on the line or at the end-wall guard,
  without touching a wall or a block.
* **ESCAPE** - with no wheel encoders, a stall is detected from the sensors
  only (flat close TOF, pressed side sonar, or flat IMU yaw while turning).
  The robot reverses while probing for room, straightens, and resumes.

All obstacle rounds use the same dead-reckoned `(progress, lateral)` frame.
At START the controller re-teleports the robot exactly onto the start line so
physics settling cannot bias the run.

---

## Open round (`mode:=open`)

The arena is a **3 m x 3 m outer square** with a **1 m x 1 m inner block**, so
the track is **1 m wide**. All walls are **10 cm high** and black. The robot
drives on the 2 m x 2 m centreline: **8 m per lap**, **24 m for 3 laps**.

Rules implemented:

* **5 s stationary start** (counted in the run time);
* **3 complete laps**, then stop on the **starting side**;
* the driving direction (**clockwise / counter-clockwise**) is assigned
  randomly for every run;
* the start position along the starting side is also random;
* no wall contact: the robot stays on the 1 m centreline.

```bash
./run_sim.sh mode:=open
./run_sim.sh mode:=open seed:=7       # repeatable random start/direction
./run_sim.sh mode:=open gui:=false rviz:=false
```

### Open-round navigation

The open round lives in the same navigator file
(`autonomous_bot_sim/color_navigator.py`) as the obstacle modes. It uses:

1. **Outer-wall following** - the outer wall is continuous around the ring.
   The side sonar holds the robot at a **0.425 m** reading, which is the
   centreline (< 0.5 m wall distance minus the 0.075 m sonar offset).
2. **Inner-block guard** - if the inner sonar gets closer than 0.255 m, the
   robot steers away from the block. This prevents corner cutting.
3. **90 deg corner turns** - when the front TOF sees the outer wall of the next
   side (`< 0.48 m`), the robot stops and turns 90 deg toward the inner block
   using the IMU. The turn direction follows the assigned lap direction.
4. **Start memorization (no line, no encoders)** - two independent references:
   * **Primary (front TOF)** - the forward TOF range at the start is the
     distance to the end wall of the starting side. After the 12th turn the
     robot is on the same side, facing the same wall, so it stops when the
     front TOF matches that remembered range (2.5 cm tolerance, 3 consecutive
     samples). This is an absolute along-track position, independent of speed.
   * **Fallback (clock)** - **A** = time from the start to the first corner
     (the partial first leg) and **B** = average time of one full side over the
     11 sides between corners 1 and 12 (turn duration excluded). If no TOF
     reference was available, the robot drives **B - A** seconds; a guard also
     stops it just past that estimate if the TOF never matches.
   * Both need no start line and no wheel encoders.
5. **Stall guard** - if the front wall stays flat much closer than the corner
   trigger, the robot backs off briefly and resumes.

The corner logic and lap counter need no wheel encoders and no world map.

### Random start / direction

`autonomous_bot_sim/open_start_randomizer.py` mirrors the event setup:

* picks one of the four sides, a random position away from the corners, and a
  random clockwise/counter-clockwise direction;
* draws no start/finish line (the real round has none; `spawn_marker:=false`);
* publishes the pose on `/open_start_pose` and the direction on
  `/open_clockwise` (both latched);
* the controller teleports the robot to that pose on START and passes the
  direction to the navigator.

Use `seed:=123` for a repeatable start; leave the default `seed:=-1` for a new
random start on every run.

### Real hardware

The randomizer is simulation-only. On the real robot, place the robot on the
start line yourself and pass the direction assigned at the event:

```bash
./run_sim.sh mode:=open open_clockwise:=true    # clockwise
./run_sim.sh mode:=open open_clockwise:=false   # counter-clockwise
```

The navigator only needs relative heading and the side sonars, so it does not
require world coordinates.

---

## Quick start (one command)

```bash
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh
```

This builds the package, sources the workspace, launches Gazebo/RViz, and
publishes `/start_button` automatically once the simulation is ready.

```bash
# obstacle rounds
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh mode:=astar
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh mode:=scan
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh mode:=bug2

# open round
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh mode:=open

# headless / manual start
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh gui:=false rviz:=false
~/ros2_ws/src/autonomous_bot_sim/run_sim.sh auto_start:=false
```

To start manually, open a second terminal and press ENTER:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run autonomous_bot_sim start_button
```

Useful launch options:

```bash
ros2 launch autonomous_bot_sim sim.launch.py mode:=open
ros2 launch autonomous_bot_sim sim.launch.py seed:=123
ros2 launch autonomous_bot_sim sim.launch.py rviz:=false gui:=false
ros2 launch autonomous_bot_sim sim.launch.py randomize:=false start_paused:=true
ros2 launch autonomous_bot_sim sim.launch.py randomize:=false \
    manual_layout:=$PWD/config/manual_example.yaml
```

## Pause and place obstacles yourself (obstacle round)

```bash
./run_sim.sh randomize:=false start_paused:=true auto_start:=false
```

Then use **Insert -> Box** in Gazebo, or spawn a YAML layout:

```bash
./run_sim.sh randomize:=false \
  manual_layout:=$PWD/config/manual_example.yaml
```

Boxes whose names start with `obstacle`, `unit_box` or `box` are published on
`/obstacle_poses` automatically. A ready-to-edit example is in
`config/manual_example.yaml`. Pause/resume from another terminal:

```bash
ros2 service call /pause_physics   std_srvs/srv/Empty
ros2 service call /unpause_physics std_srvs/srv/Empty
```

## Terminal output

When the run completes the controller prints the total challenge time:

```
==============================================================
  CHALLENGE COMPLETE in 32.80 seconds
  reason: finish line crossed (safe stop before wall)
==============================================================
```

For the open round the reason is, for example:

```
  reason: open round complete: 3 laps (22.55 m)
```

The elapsed time always includes the 5 s start hold. If you press **Ctrl+C**
early, the controller prints the elapsed time from START to the stop instant.

## Key constants

Obstacle-round (`autonomous_bot_sim/color_navigator.py`):

| Constant | Value | Meaning |
|---|---|---|
| `WALL_P` | 2.825 m | end-wall inner face from the start pose |
| `WALL_SIDE` | 0.475 m | side-wall inner face from the lane centre |
| `FINISH_P` | 2.525 m | dead-reckoned finish target (`finish_y = 1.175`) |
| `PLAN_CLEAR` | 0.125 m | guaranteed free radius used by the planner |
| `DECIDE_DIST` | 0.160 m | stop this far before a block |
| `T_FINAL` | 30.0 s | FINAL approach timeout |

Open-round:

| Constant | Value | Meaning |
|---|---|---|
| `OPEN_LAPS` | 3 | complete laps |
| `OPEN_SIDE_LEN` | 2.0 m | centreline side length |
| `OPEN_LAP_LEN` | 8.0 m | centreline distance per lap |
| `OPEN_TOTAL` | 24.0 m | fallback distance for 3 laps |
| `OPEN_SIDE_TARGET` | 0.425 m | sonar reading on the track centreline |
| `OPEN_CORNER_FRONT` | 0.48 m | front TOF range that triggers a corner turn |
| `OPEN_V` | 0.130 m/s | open-round cruise speed |

Simulated ray sensors (`urdf/robot.urdf.xacro`):

| Constant | Value | Meaning |
|---|---|---|
| `ray_max` | 5.5 m | ray length, longer than either world diagonal (avoids `FLT_MAX`) |

## Directory layout

* `autonomous_bot_sim/arena_layout.py` - obstacle scatter, gap/BFS validation
* `autonomous_bot_sim/obstacle_randomizer.py` - spawns the blocks and
  publishes `/obstacle_poses`
* `autonomous_bot_sim/open_start_randomizer.py` - random open-round start,
  direction and start/finish marker
* `autonomous_bot_sim/manual_obstacles.py` - manual mode: spawns a YAML
  layout and mirrors hand-placed boxes onto `/obstacle_poses`
* `autonomous_bot_sim/color_navigator.py` - all four modes (`astar`, `scan`,
  `bug2`, `open`) in one state machine
* `autonomous_bot_sim/autonomous_controller.py` - ROS wrapper, vision, timer,
  start teleport, dead reckoning
* `config/manual_example.yaml` - example manual obstacle layout
* `urdf/robot.urdf.xacro` - angled camera, servo/TOF, left/right sonars
* `worlds/obstacle_round.world` - 1 x 3 m obstacle round
* `worlds/open_round.world` - 3 x 3 m open round
