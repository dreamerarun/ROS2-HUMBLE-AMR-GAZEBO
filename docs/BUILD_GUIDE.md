# Building a Diff-Drive Robot in ROS2 Humble + Gazebo Harmonic
### A complete guide: URDF → Sensors → Gazebo → SLAM → Nav2 → Waypoints

This document walks through everything built in this project, in order, with
the reasoning behind each piece of code and a catalog of every error hit
along the way — what caused it, and why the fix works. Package name throughout:
`my_robot_description`, workspace: `~/ros_ws`.

---

## 1. URDF/Xacro basics — building the robot from scratch

### Why xacro instead of raw URDF
Raw URDF has no variables, no macros, no math — every wheel, every link needs
to be hand-written in full. Xacro adds `<xacro:property>` (variables),
`<xacro:macro>` (reusable templates), and math expressions like `${pi/2}`.
This is why the wheel macro could be written once and instantiated twice
(left/right) instead of copy-pasted.

### Core structure
- `base_link` — a massless root link (URDF convention; the "real" body starts
  one joint down)
- `chassis` — a box link (visual + collision + inertial), fixed-jointed to
  `base_link`
- **Wheel macro** (`xacro:macro name="wheel" params="prefix x_reflect y_reflect"`)
  — defines a cylinder link + a `continuous` joint (unlimited rotation, unlike
  `revolute` which needs angle limits) with `axis xyz="0 1 0"` so it spins on
  its own Y-axis. Instantiated as `left_wheel` / `right_wheel` by flipping
  the sign of the Y offset.
- **Caster wheel** — a sphere, `fixed` joint, zero friction (`mu1`/`mu2` = 0
  in the Gazebo block later) since it's a passive support point, not driven.

### Key URDF/xacro lesson: link lumping
When URDF is converted to SDF for Gazebo, **fixed joints get "lumped"** —
child links connected by `type="fixed"` joints get merged into their parent
in the exported SDF, for physics-engine efficiency. This silently broke our
lidar/camera frame names later (see Error #7 below). The fix is
`<disableFixedJointLumping>true</disableFixedJointLumping>` inside a
`<gazebo reference="joint_name">` block for any fixed joint whose child link
needs to stay a distinct frame (like a sensor mount).

---

## 2. Gazebo Harmonic integration

### Materials & friction
Plain URDF `<material>` tags don't apply in `gz sim` — Gazebo reads separate
`<gazebo reference="link_name">` blocks:
```xml
<gazebo reference="left_wheel">
  <mu1>1.0</mu1>
  <mu2>1.0</mu2>
  <material>Gazebo/Black</material>
</gazebo>
```
High friction on drive wheels = grip (torque becomes motion). Zero friction
on the caster = it slides freely instead of dragging.

### The DiffDrive plugin
This is what turns wheel joints into an actually-drivable robot:
```xml
<gazebo>
  <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>${2*(chassis_width/2 + wheel_ygap)}</wheel_separation>
    <wheel_radius>${wheel_radius}</wheel_radius>
    <topic>cmd_vel</topic>
    <odom_topic>odom</odom_topic>
    <tf_topic>/tf</tf_topic>
    <frame_id>odom</frame_id>
    <child_frame_id>base_link</child_frame_id>
  </plugin>
  <plugin filename="gz-sim-joint-state-publisher-system"
          name="gz::sim::systems::JointStatePublisher">
    <topic>joint_states</topic>
  </plugin>
</gazebo>
```
It subscribes to `Twist` on `cmd_vel` and converts linear/angular velocity
into left/right wheel speeds internally.

### The world file
Built from scratch with: physics/sensors/scene-broadcaster plugins, a
directional light, a manually-defined `ground_plane` model (see Error #3),
3 obstacle boxes, and 4 perimeter walls (so SLAM/Nav2 would have a bounded
space to map).

---

## 3. Sensors — lidar and camera

Added as `<link>` + `<joint>` (fixed-mounted to chassis) plus a
`<gazebo reference="...">` block containing a `<sensor>` tag:
- **Lidar**: `type="gpu_lidar"`, 360° horizontal sweep, `range_min=0.12`,
  `range_max=10.0`
- **Camera**: `type="camera"`, 640x480, standard pinhole FOV

---

## 4. The launch file (`gazebo.launch.py`)

Five pieces, in order:
1. **`gazebo`** — includes `ros_gz_sim`'s `gz_sim.launch.py`, passing our
   world file
2. **`robot_state_publisher`** — runs `xacro` at launch time via
   `ParameterValue(Command(['xacro ', xacro_file]), value_type=str)`,
   publishes TF from the resulting URDF
3. **`spawn_robot`** — `ros_gz_sim create -topic robot_description` reads
   the URDF that `robot_state_publisher` just published and spawns it into
   the running world
4. **`bridge`** — `ros_gz_bridge parameter_bridge`, translating topics
   between ROS2 and Gazebo Transport (`@`=ROS msg type, `]`=ROS→GZ,
   `[`=GZ→ROS)
5. **`lidar_frame_bridge`** — a `static_transform_publisher` connecting the
   URDF's `lidar_link` frame to the frame name the *bridged* LaserScan
   message actually carries (see Error #8)

---

## 5. Every error hit, and why the fix works

### Error 1 — `xacro: file not found ... math.xacro`
**Cause:** an unnecessary `xacro:include` for a math library that isn't a
real bundled file in modern xacro.
**Fix:** delete the include entirely — xacro ≥2.1.1 has `pi` built in as a
constant.

### Error 2 — `UnboundLocalError: local variable 'Node' referenced before assignment`
**Cause:** a second `from launch_ros.actions import Node` was added *inside*
the `generate_launch_description()` function body, after `Node(...)` was
already called earlier in the same function. Python sees any assignment to a
name anywhere in a function and treats that name as local for the *entire*
function scope — including calls before the reassignment.
**Fix:** one import at the top of the file only; never re-import inside the
function.

### Error 3 — `Unable to find or download file` (Gazebo world fails to load)
**Cause:** the world file's `<include><uri>https://fuel.gazebosim.org/...
Ground Plane</uri></include>` tries to fetch a model from the internet at
launch time — fails with no/unreliable connectivity.
**Fix:** define the ground plane directly in SDF as a `<model>` with a
`<plane>` geometry — zero network dependency.

### Error 4 — World edits not taking effect after rebuild
**Cause:** `worlds/` directory wasn't registered in the package's
`install()` rule (CMakeLists.txt or `setup.py`'s `data_files`), so colcon
never copied edited files into the installed share directory being launched
from.
**Fix:** add every asset directory (`urdf`, `launch`, `worlds`, `rviz`,
`config`, `maps`) to the install rule. When in doubt, `rm -rf build/<pkg>
install/<pkg>` and rebuild clean to force re-registration of a newly added
directory.

### Error 5 — Wheels have no TF / missing from RobotModel in RViz
**Cause:** `continuous` joints (unlike `fixed` ones) need live joint-state
data for `robot_state_publisher` to compute their transform. Gazebo's
`JointStatePublisher` plugin published this data *inside* Gazebo, but the
bridge never relayed `/joint_states` to ROS2 — it was missing from the
bridge's `arguments` list.
**Fix:** add
`'/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'` to the bridge.
This bug recurred once later when a launch-file rewrite (fixing Error #2)
accidentally reverted from an older saved copy — a good reminder to re-check
the *whole* file after any edit, not just the part you touched.

### Error 6 — Lidar publishes all `inf` / EGL rendering warnings
**Cause:** `gpu_lidar` needs a real GPU rendering context even in a
headless/simulated setting. On an NVIDIA Optimus laptop,
`LIBGL_ALWAYS_SOFTWARE=1` alone conflicts with EGL's explicit hardware
device selection (`Not allowed to force software rendering when API
explicitly selects a hardware device`).
**Fix:** force the real NVIDIA GPU to render instead of fighting for
software fallback:
```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __VK_LAYER_NV_optimus=NVIDIA_only
```

### Error 7 — Lidar frame reports as `base_link` instead of `lidar_link`
**Cause:** URDF→SDF fixed-joint lumping (see section 1) merged `chassis` and
`lidar_link` together since they're connected via `type="fixed"` joints, so
Gazebo named the sensor's frame using the surviving link name.
**Fix:** `<disableFixedJointLumping>true</disableFixedJointLumping>` (plus
`<preserveFixedJoint>true</preserveFixedJoint>` for broader version
compatibility) inside a `<gazebo reference="lidar_joint">` block.

### Error 8 — Lidar frame is `my_robot::lidar_link::lidar` in Gazebo but TF lookups fail
**Cause:** `ros_gz_bridge` **sanitizes** Gazebo's scoped entity names when
converting to ROS2 messages — ROS TF frame names can't contain `::`, so the
bridge silently rewrites `::` to `/`. The *actual* ROS-side frame_id was
`my_robot/lidar_link/lidar`, not `my_robot::lidar_link::lidar`. A static
transform published with the wrong (colon) child-frame name matched nothing
real, so RViz's TF lookup for every scan point failed silently.
**Fix:** confirm the real frame name directly —
`ros2 topic echo /scan --once` and read the literal `frame_id` field, don't
assume it matches the Gazebo-side name from `gz topic -e`. Point the
`static_transform_publisher`'s `--child-frame-id` at that exact string.

### Error 9 — RViz LaserScan/Map shows nothing despite genuinely live data
This happened repeatedly, for a few different underlying reasons that all
produce the same symptom (topic publishing + TF fine + still nothing
rendered):
- **QoS Durability mismatch**: `map_server` publishes `/map` as
  **Transient Local** so late-joining subscribers still get the last
  published map; RViz's default is **Volatile**, which only receives
  messages published *after* subscribing. Fix: manually set the display's
  Topic → Durability Policy to `Transient Local`.
- **Wrong display Style/Size units**: `Points` style sizes are in **screen
  pixels** (3px ≈ invisible); `Boxes` style at meter-scale sizes (0.8m) on a
  40cm robot fuses hundreds of overlapping points into one blob. Fix:
  `Flat Squares` style with `Size (m)` around `0.05`.
- **Wrong Topic entirely**: a display was pointed at
  `/local_costmap/costmap` or `/global_costmap/costmap` instead of `/map`
  after being re-added — always double check the Topic field after Add.
- **Unsaved RViz config**: the window title showing `robot.rviz*` (asterisk)
  means live GUI fixes were never written to disk, so every relaunch
  reloaded the old broken config. Fix: **File → Save Config**, or — more
  reliably — write the `.rviz` YAML file directly and `cp` it into both the
  `src/` and installed `share/` paths so there's no ambiguity.
- **Stuck display instance**: after several live property edits in one
  session, a display can get into a state where its Color/Position
  Transformer dropdowns are empty even though Status says "Ok." Fix: Remove
  the display and Add a fresh one rather than keep editing the stuck one.

### Error 10 — `RViz Fixed Frame = base_link` makes the robot look stationary
**Cause:** Fixed Frame defines what RViz treats as the static world origin.
With it set to `base_link`, the robot is *by definition* always drawn at the
center — everything else (map, TF markers, odom) appears to move around it
instead. This isn't a bug, just a different (egocentric) view mode.
**Fix:** set Fixed Frame to `odom` (for driving around before a map exists)
or `map` (once SLAM/Nav2 is running) to get the normal "watch the robot move
through the world" view.

### Error 11 — `map_server`: `parameter 'yaml_filename' is not initialized`
**Cause:** Nav2's `bringup_launch.py` injects the map path from the `map`
launch argument into the params file at runtime via `RewrittenYaml` — but
this mechanism can only **override an existing key**, it can't create a
missing section. Our `nav2_params.yaml` had no `map_server:` section at all.
**Fix:** add an explicit `map_server:` section with a placeholder
`yaml_filename: ""` so there's something to override.

### Error 12 — `Failed to create global planner... class does not exist`
**Cause:** pluginlib lookup names for `nav2_navfn_planner` use the
`package/ClassName` slash format (`nav2_navfn_planner/NavfnPlanner`), not
C++ namespace syntax (`nav2_navfn_planner::NavfnPlanner`). The error
message's own "Declared types are..." list is the authoritative source of
truth for the correct string.
**Fix:** use the exact string from that declared-types list.

### Error 13 — Same bug, `behavior_server` this time
**Cause:** identical issue — `nav2_behaviors::Spin` needed to be
`nav2_behaviors/Spin` (and `BackUp`, `Wait` likewise). Note this is
*inconsistent* across Nav2 plugin families — `dwb_core::DWBLocalPlanner` and
`nav2_controller::SimpleProgressChecker` correctly use `::`, while
`nav2_navfn_planner` and `nav2_behaviors` need `/`. There's no shortcut here
except reading each error's declared-types list carefully.

### Error 14 — `bt_navigator`: `Couldn't open input XML file: navigate_to_pose_w_replanning_and_recovery.xml`
**Cause:** `default_nav_to_pose_bt_xml` was set to a bare filename;
`bt_navigator` doesn't search any default directory for it — it needs a
full filesystem path.
**Fix:** point it at the actual installed file, e.g.
`/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
(verify the exact path with `find /opt/ros/humble -iname
"*navigate_to_pose*replanning*"` since it can vary by install).

### Error 15 — `Unable to start transition 1/4 from current state active/inactive: Transition is not registered`
**Cause:** a stale Nav2 process from a previous launch was still running
when a new `navigation.launch.py` was started, so the lifecycle manager
tried to configure/activate nodes that were already in a different state
than expected.
**Fix:** `pkill -9 -f component_container` before every relaunch during
active debugging, to guarantee a clean process slate.

### Error 16 — `Timed out waiting for transform from base_link to map` / AMCL `extrapolation into the future`
**Cause:** this is a *downstream symptom*, not an independent bug — it
happens whenever the navigation lifecycle group fails to fully activate
(see Errors #11-15), leaving TF publishing incomplete/stalled while AMCL
and costmaps wait on data that never arrives on schedule. Once the actual
root-cause config bug is fixed and the full stack activates cleanly, this
error disappears on its own.

### Error 17 — `2D Goal Pose` / `Nav2 Goal` → "Goal was rejected by server"
**Cause:** `bt_navigator` (which hosts the `NavigateToPose` action server)
was stuck at `inactive` due to Error #14 — there was nothing listening for
goals at all.
**Fix:** same as Error #14; once `bt_navigator` reaches `active [3]`, goals
are accepted normally.

---

## 6. SLAM (slam_toolbox)

`slam_toolbox`'s `online_async_launch.py` is included with our own
`slam_toolbox_params.yaml` (solver: Ceres, `mode: mapping`,
`resolution: 0.05`). Drive the robot around with teleop while it's running;
`slam_toolbox` publishes `/map` and the `map → odom` TF. Save with:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros_ws/src/my_robot_description/maps/my_map
```
This writes a `.pgm` (image) + `.yaml` (metadata: resolution, origin,
thresholds) pair.

## 7. Nav2 autonomous navigation

`navigation.launch.py` includes `nav2_bringup`'s `bringup_launch.py`,
passing our map file and `nav2_params.yaml`. This starts two lifecycle
manager groups:
- **localization**: `map_server`, `amcl`
- **navigation**: `controller_server`, `smoother_server`, `planner_server`,
  `behavior_server`, `bt_navigator`, `waypoint_follower`,
  `velocity_smoother`

Each group configures every node, then activates every node, in sequence —
if any single node fails either step, the whole group aborts (this is why
one bad plugin string cascaded into "the entire nav stack is broken").

**Workflow**: 2D Pose Estimate (click-hold-drag-release on the map to seed
AMCL's particle filter) → wait for `/amcl_pose` to show a real, converged
pose → 2D Goal Pose or Nav2 Goal to send a destination → watch `/plan`
populate and `/cmd_vel` start ticking.

## 8. Waypoint navigation

Two approaches:
- **RViz's Navigation 2 panel** → "Waypoint / Nav Through Poses Mode"
  button → click multiple points on the map → Start Navigation. Good for
  quick manual tests.
- **`nav2_simple_commander`** (Python) → `BasicNavigator().followWaypoints
  ([...])`, sending a list of `PoseStamped` goals as one `FollowWaypoints`
  action call. Reusable/scriptable — the basis for real patrol/inspection
  routines.

---

## The one meta-lesson across all of this

Nearly every hard-to-diagnose bug in this project came down to one of three
categories, and it's worth internalizing them for future ROS2/Gazebo work:

1. **A name string mismatch** (frame_id after bridge sanitization, pluginlib
   `::` vs `/` lookup format, a bare filename vs full path) — the fix is
   almost always to *read the exact string the error or raw topic gives
   you*, rather than assume convention.
2. **A config or file change that "should" have taken effect but didn't**
   (missing install() rule, unsaved RViz config, stale running process) —
   the fix is to verify the *actual current state* directly
   (`ros2 lifecycle get`, `ros2 topic echo --once`, `cat` the installed
   file) rather than trust that an edit propagated.
3. **QoS/lifecycle sequencing** (Transient Local vs Volatile, one lifecycle
   node failing aborting the whole group) — understanding *why* these
   systems are designed this way (late-joining subscribers, atomic group
   bring-up) makes the errors much faster to diagnose next time.
