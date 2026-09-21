# NoMagic: native ROS1 support in depthai_ros_driver_v3

The driver can additionally speak ROS1 ("ROS One", the community ROS1 distribution for
Ubuntu 22.04) natively from the same process — the robot-driver "pattern A" from the NoMagic
ROS1_AND_2_SUPPORT document. Images are published on ROS1 directly (no `ros1_bridge` in the
image path); TFs, if ever enabled, remain a job for the bridge.

## What the ROS1 side provides

With a v2.11.2-noetic-equivalent config (see `nomagic/` configs) and a ROS master running:

- `/<node>/rgb/image_raw`, `/<node>/rgb/camera_info`,
  `/<node>/stereo/image_raw`, `/<node>/stereo/camera_info` — plain `ros::Publisher`s fed with
  the very same messages the ROS2 publishers get (converted field-by-field; `header.seq` = 0).
  Lazy publishing (`i_enable_lazy_publisher`) is evaluated per graph: each side publishes when
  *it* has subscribers.
- `/<node>/start_camera`, `/<node>/stop_camera`, `/<node>/save_pipeline`,
  `/<node>/save_calibration` — `std_srvs/Trigger`, the v2.x ROS1 service names (the ROS2 graph
  keeps the v3 names `start_driver`/`stop_driver`/...). Note: v2.11.2 mis-bound
  save_pipeline/save_calibration to the start/stop callbacks; here they do what they say.

Not provided on ROS1 (documented differences): image_transport subtopics
(`.../compressed|compressedDepth|theora`), `set_camera_info`, dynamic_reconfigure, TF.
Reason: ROS1 `image_transport`/`camera_info_manager`/`tf2_ros`/`dynamic_reconfigure` share
sonames with (or drag in libraries clashing with) their ROS2 twins already linked into this
process, so the ROS1 side sticks to `roscpp` + message headers only.

## Runtime behavior

- ROS1 comes up lazily in `Driver`'s start timer with the same node name as the rclcpp node.
- If the env var `NOMAGIC_ROS1_ENABLE=0` is set, or no ROS master is reachable at
  `ROS_MASTER_URI` at startup, the ROS1 side stays off and the driver behaves like stock ROS2.
- `rclcpp::shutdown` (SIGINT) also shuts the ROS1 side down.
- ROS1 callbacks run on a dedicated `ros::AsyncSpinner(1)`.

## Building

```bash
source /opt/ros/one/setup.bash        # ROS1 first (provides PKG_CONFIG_PATH for find_ros1_package)
source /opt/ros/humble/setup.bash     # ROS2 last (ROS_VERSION=2 wins for colcon/rosidl)
colcon build --packages-select depthai_ros_driver_v3 --cmake-args -DNOMAGIC_ROS1=ON
```

With `-DNOMAGIC_ROS1=OFF` (default) the build is byte-identical to upstream.

## Rebase surface (kept deliberately small)

New files (never conflict):
- `depthai_ros_driver/cmake/find_ros1_package.cmake` (verbatim from ros2/ros1_bridge)
- `depthai_ros_driver/include/depthai_ros_driver_v3/ros1/ros1_node.hpp`
- `depthai_ros_driver/src/ros1/ros1_node.cpp`
- this document

Hooks in upstream files (all guarded by `#ifdef NOMAGIC_ROS1` / `if(NOMAGIC_ROS1)`):
1. `CMakeLists.txt`: one `option()` + `if(NOMAGIC_ROS1)` block after the target definitions.
2. `include/.../img_pub.hpp`: one include + one `ros1Pub` member.
3. `src/dai_nodes/sensors/img_pub.cpp`: advertise in `setup()` (non-compressed branch),
   publish in `publish(std::shared_ptr<Image>)` before the ROS2 publish (which may move the
   message out).
4. `src/driver.cpp`: one include; `Ros1Node::init(get_name())` at the top of the start timer;
   the four `advertiseTrigger` calls next to the ROS2 service registrations.

Unrelated fix carried in this fork: `Driver::startDevice()` accepts `X_LINK_ANY_STATE` in the
connect-by-IP fallback so unicast-only networks (no broadcast discovery) work.
