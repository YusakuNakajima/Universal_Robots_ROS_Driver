# Joint Torque Control

This document describes how to use the joint torque control interface of the `ur_robot_driver`.

**WARNING:** This is an advanced feature that gives you direct control over the robot's joint torques. Incorrect use can lead to high accelerations and unpredictable or unstable behavior. Use this feature with caution, start with low torque values, and always be prepared to use the emergency stop. This feature is only available on e-Series robots.

## How it Works

The torque control implementation relies on a combination of a dedicated URScript, the RTDE interface, and `ros_control`.

1.  **URScript:** A specific URScript (`resources/torque_control.urscript`) is sent to the robot controller. This script runs a loop that continuously reads target torque values from 6 RTDE input registers.
2.  **`torque_command`:** The script passes the received values directly to the `torque_command()` function inside the robot's controller. This function commands the robot to apply the specified torques at the next control cycle.
3.  **RTDE Interface:** The ROS driver writes the desired joint torques to the RTDE input registers that the URScript is reading from. A dedicated recipe (`resources/rtde_input_torque_recipe.txt`) is used for this.
4.  **`ros_control`:** The `hardware_interface` has been extended to support the `hardware_interface::EffortJointInterface`. This allows standard ROS controllers, such as `effort_controllers/JointGroupEffortController`, to send torque commands to the hardware interface, which then forwards them to the robot via RTDE.

## How to Use Torque Control

### 1. Launch the Torque Control Driver

To enable the torque control interface, use the dedicated launch file `ur_torque_control.launch`. This launch file loads the correct URScript, RTDE recipes, and the necessary `ros_control` controllers.

You will need to provide your robot's IP address and the path to its kinematics calibration file.

```bash
roslaunch ur_robot_driver ur_torque_control.launch robot_ip:=<your_robot_ip> kinematics_config:=<path_to_your_calibration_file>
```

This will start the driver and load the `joint_group_effort_controller`.

### 2. Send Torque Commands

Once the driver is running, you can send torque commands by publishing to the `/joint_group_effort_controller/command` topic. The message type is `std_msgs/Float64MultiArray`.

The `data` field of the message should be an array of 6 floating-point numbers, representing the target torques in Nm for each joint, in the standard order:
`[shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]`

**Example:** Apply a constant torque of 2.5 Nm to the elbow joint.

```bash
rostopic pub /joint_group_effort_controller/command std_msgs/Float64MultiArray "layout:
  dim: []
  data_offset: 0
data: [0.0, 0.0, 2.5, 0.0, 0.0, 0.0]" -1
```

**Example:** Command zero torque to all joints (this should make the robot compliant, but be aware of gravity).

```bash
rostopic pub /joint_group_effort_controller/command std_msgs/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" -1
```

### 3. Implementing a Custom Torque Controller

You can create your own controller node in ROS that implements more advanced logic (e.g., PD control, gravity compensation, etc.).

A typical structure for such a controller would be:
1.  **Subscribe** to the `/joint_states` topic to get the current joint positions and velocities.
2.  **Calculate** the desired torques based on your control algorithm (e.g., `torque = Kp * (pos_desired - pos_actual) + Kd * (vel_desired - vel_actual)`).
3.  **Publish** the calculated torques as a `std_msgs/Float64MultiArray` message to the `/joint_group_effort_controller/command` topic.
