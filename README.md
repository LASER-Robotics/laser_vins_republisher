# LASER VINS Republisher

`laser_vins_republisher` converts OpenVINS odometry into the FCU and odometry frames used by the LASER UAV System (LUS). It applies the camera/IMU mounting transform, rotates velocities and covariances, and publishes the result for `estimation_manager`.

The package provides a ROS 2 lifecycle node. Its launch file configures and activates it automatically.

## Run

```bash
ros2 launch laser_vins_republisher vins_republisher.launch.py namespace:=uav1
```

Start OpenVINS separately. The supplied launch connects to its `ov_msckf/odomimu` output in the same namespace.

## Interfaces

For `namespace:=uav1`:

| Direction | Topic | Type | Purpose |
| --- | --- | --- | --- |
| Input | `/uav1/ov_msckf/odomimu` | `nav_msgs/msg/Odometry` | Raw visual-inertial state. |
| Output | `/uav1/vins_republisher/odometry` | `nav_msgs/msg/Odometry` | Transformed pose, velocity and covariance. |
| Output | `/tf_static` | `tf2_msgs/msg/TFMessage` | Configured FCU-to-VINS mounting transform. |
| Output | `/tf` | `tf2_msgs/msg/TFMessage` | Dynamic frame relationships used by the republisher. |

The node-level input and output names are `odometry_in` and `odometry_out`. The implementation does not publish a `path` topic. It requires the mounting transform at the input timestamp before publishing odometry.

## Configuration

Use [params/vins_republisher.yaml](params/vins_republisher.yaml), or select another file with `vins_republisher_params_file`.

| Parameter | Supplied value | Meaning |
| --- | --- | --- |
| `velocity_in_body_frame` | `true` | Input velocities are already in the body frame. Set `false` for an input that needs world-to-body rotation. |
| `init_in_zero` | `true` | Removes the initial heading offset. |
| `compensate_initial_tilt` | `false` | Enables the initial tilt-calibration path. Keep disabled in this checkout: it waits for calibration, but no calibration service is created. |
| `rate_limiter.enabled` | `false` | Enables output rate limiting. |
| `rate_limiter.max_rate` | `100.0` | Maximum processing rate in Hz when limiting is enabled. |
| `vins_world_frame` | `vins_world` | Output odometry frame suffix. |
| `fcu_frame` | `fcu` | FCU frame suffix. |
| `vins_frame` | `ov_imu` | VINS IMU frame suffix. |
| `static_transform.translation.{x,y,z}` | `0.0` on each axis | Mounting translation in metres. |
| `static_transform.rotation.{roll,pitch,yaw}` | `-1.570796`, `0.0`, `-1.570796` | Mounting rotation in radians. |

The node prefixes the configured frame suffixes with `uav_name`, supplied by the launch from `namespace`. For example, `fcu` becomes `uav1/fcu`. Use the measured mounting transform for your vehicle.

| Launch argument | Default | Meaning |
| --- | --- | --- |
| `namespace` | `UAV_NAME`, or `uav1` | Node namespace and `uav_name`. |
| `use_sim_time` | `true` when `REAL_UAV=false` | Uses the simulation clock. |
| `vins_republisher_params_file` | Installed `params/vins_republisher.yaml` | Full path to the parameter file. |
