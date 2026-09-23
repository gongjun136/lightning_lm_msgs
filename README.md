# common_msgs

Shared ROS 2 message and service packages used by Lightning localization,
Livox LiDAR integration, and vehicle-side applications.

## Repository layout

- `livox_ros_driver/`: package `livox_ros_driver2`; Livox message definitions only.
- `diagnostic_monitor_interfaces/`: functional-safety heartbeat interfaces.
- `lightning_interfaces/`: Lightning localization interfaces.
- Other directories contain shared project-specific interfaces.

This repository contains interfaces only. Driver and algorithm implementations
must depend on these packages instead of embedding duplicate message definitions.

## Workspace import

This repository is normally imported beside its consumers with a `.repos` file:

```bash
vcs import src < src/<consumer-repository>/common_msgs.repos
```

Each ROS package name must appear only once in a colcon workspace.
