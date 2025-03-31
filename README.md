# Assignment 2 - Part 1

### You can click here to have a look to the documentation
   https://l-xiii.github.io/assignment_2_2024/
   
## Overview
This part of the assignment implements key components of a robot simulation in ROS, focusing on:

1. **Action Client Node**: Allows a user to set targets and interact with an action server.
2. **Service Node**: Provides a service to retrieve the last target sent by the action client.
3. **Custom ROS Interfaces**: Includes a custom message and a service definition.
4. **Launch File**: Simplifies starting the simulation by launching multiple nodes.

---

## Files

### 1. `action_client_node.cpp`
This file contains the implementation of the **Action Client Node**. Its primary features include:
- Sending targets (x, y coordinates) to an action server.
- Publishing robot position and velocity to the `/PoseVel_state` topic.
- Using feedback from the action server to determine when a goal has been reached.
- Command-line interface for setting or canceling targets.

#### Topics:
- **Publish**:
  - `/PoseVel_state`: Publishes the robot's current position (`x`, `y`) and velocity (`vel_x`, `vel_z`).
- **Subscribe**:
  - `/odom`: Subscribes to odometry data to track the robot's movement.

---

### 2. `service_node.cpp`
This file implements the **Service Node**, which provides a service for retrieving the last target sent by the Action Client Node.

#### Service:
- **Name**: `/Get_Last_Target`
- **Definition**: `GetLastTarget.srv`
- **Functionality**:
  - Responds with the coordinates (`x`, `y`) of the most recent target set by the Action Client Node.

---

### 3. `GetLastTarget.srv`
This is the custom service definition used by the `service_node`. It defines:
- **Request**: No input is required.
- **Response**:
  - `x`: The x-coordinate of the last target.
  - `y`: The y-coordinate of the last target.

---

### 4. `PoseVel.msg`
This is the custom message type used by the Action Client Node to publish robot position and velocity.

#### Fields:
- `float64 x`: Robot's x-coordinate.
- `float64 y`: Robot's y-coordinate.
- `float64 vel_x`: Linear velocity of the robot.
- `float64 vel_z`: Angular velocity of the robot.

---

### 5. `start_simulation.launch`
This is the main launch file for running the nodes.

#### Components:
- **Service Node**: Launches the `service_node`.
- **Action Client Node**: Executes the `action_client_node` via a separate terminal (handled by a custom script or `launch-prefix`).
- Includes parameters and other necessary components for the simulation.

---

## Usage

### Prerequisites
Ensure that the custom message (`PoseVel.msg`) and service (`GetLastTarget.srv`) are built and sourced correctly:

```bash
catkin_make
source devel/setup.bash
