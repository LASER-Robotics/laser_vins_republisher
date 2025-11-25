# LASER VINS Republisher

This package provides a **ROS 2 node** that acts as a bridge between the VINS-Fusion state estimator and the Laser UAV System (LUS).

## Overview

The main objective of this package is to process the odometry data coming from visual-inertial odometry (VIO) systems, specifically OPENVins. It handles frame transformations, velocity rotations, and initialization resets to ensure the odometry is compatible with the downstream EKF (e.g., `laser_uav_estimators`).

### Key Functionalities
1.  **Frame Transformation:** Applies a static transform between the Flight Controller (FCU) frame and the VINS IMU frame.
2.  **Velocity Adjustment:** Can transform velocities from the global frame to the body frame if required.
3.  **Initialization Handling:** Options to reset the initial heading to zero or compensate for initial tilt, ensuring a consistent starting state.
4.  **Rate Limiting:** Optional downsampling of the odometry output to save computational resources.

## Provided Nodes

### 1. `vins_republisher_node`
-   **Description:** This node subscribes to the raw odometry from VINS, applies coordinate transformations, handles frame alignments, and republishes the data for the estimator.

-   **Subscribed Topics:**
    -   `/vins_estimator/odometry`: The raw odometry output from VINS-Fusion.

-   **Published Topics:**
    -   `~/odometry`: The processed odometry ready for sensor fusion.
    -   `~/path`: The trajectory path of the UAV for visualization.

-   **Configurable Parameters:**
    ```yaml
    vins_republisher_node:
      ros__parameters:
        # VINS Frame Settings
        vins_world_frame: 'vins_world'
        fcu_frame: 'fcu'       # The frame ID of the flight controller
        vins_frame: 'ov_imu'   # The frame ID of the VINS IMU

        # Velocity Handling
        # OpenVins typically publishes velocities in the body frame.
        # If false, the node transforms global frame velocities to the body frame.
        velocity_in_body_frame: true

        # Initialization
        # Subtracts the initial orientation so the output starts at zero (identity quaternion).
        # Useful because VINS often initializes with an arbitrary yaw.
        init_in_zero: true

        # If true, waits for a service call to calibrate the level horizon (zero pitch/roll).
        # Helps if VINS initialized while the drone was not perfectly level.
        compensate_initial_tilt: false

        # Rate Limiter
        # Limits the output frequency to save resources (e.g., if VINS runs at camera FPS).
        rate_limiter:
          enabled: false
          max_rate: 100.0 # Hz

        # Static Transform (Rotation/Translation from FCU to VINS Frame)
        # This defines the physical rotation between the controller definition and VINS definition.
        static_transform:
          translation:
            x: 0.0
            y: 0.0
            z: 0.0
          rotation:
            # Example: -90 degrees roll and -90 degrees yaw
            roll: -1.570796
            pitch: 0.0
            yaw: -1.570796
    ```