#!/usr/bin/env bash

TOPICS=(
    # Always keep these
    /clock
    /tf
    /tf_static
    /parameter_events
    /rosout

    # Microstrain IMU
    /imu/data
    /imu/data_raw
    /ekf/status

    # Livox LiDAR and IMU
    /livox/imu
    /livox/points

    # Radio telemetry
    /telem

    # DLIO Odom and mapping
    /dlio/odom_node/keyframes
    /dlio/odom_node/odom
    /dlio/odom_node/path
    /dlio/odom_node/pose
    /dlio/odom_body
    /map
    # /dlio/odom_node/pointcloud/deskewed
    # /dlio/odom_node/pointcloud/keyframe
)

ros2 bag record "${TOPICS[@]}"