# drake-ur-driver

### Setup

#### 1. Robot Setup


 https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/setup/robot_setup.html

 Note: It is SSH into the controller with username `root@<ip address of controller>` and password `easybot`

#### 2. Network Setup


https://docs.universal-robots.com/Universal_Robots_ROS_Documentation/doc/ur_client_library/doc/setup/network_setup.html

Note: It's not required to have the controller directly connected to the control PC

#### 3. Running the driver

```
bazel run //drake-ur-driver:ur_driver -- <3.1 running options>
```

running options are specified in `x=y` format. Where **x** are the keys as detailed before, and **y** is the corresponding values.

##### 3.1 Running Options

`robot_ip_address` : robot controller IP Address (default: "192.168.56.101")

`lcm_url` : LCM URL for UR driver (default: empty string for default LCM)

`lcm_command_channel` : name of the channel on which to listen to command messages of type *lcmt_ur_command* (default: "UR_COMMAND")

`lcm_status_channel` : name of the channel on which to publish status messages of type *lcmt_ur_status* (default: "UR_STATUS")

`control_mode` : control mode for the robot. Choose from: **status_only**, **velocity**, **position** (default), **tcp_position**, **tcp_velocity**

`use_mbp` : use Drake MultibodyPlant for dynamics computation and limit checks (default: false). If true, use `urdf_path` to provide the model. If false, skip limit checks.

`urdf_path` : path to the URDF model file to use for limit checking (default: "drake-ur-driver/resources/ur10.urdf")

`scale_velocity_limits` : velocity joint limit factor k where k * joint_velocity_limit / 100.0. Range [1-100]. Use -1 to disable (default: -1)

`scale_joint_limits` : joint limit factor k where k * joint_limit / 100.0. Range [1-100]. Use -1 to disable (default: -1)

`command_stop_limit` : maximum number of consecutive missed messages before stopping commands to robot (default: 5)

`expire_sec` : maximum allowed delay (in seconds) for messages to be considered valid. Must be non-negative and finite (default: 0.1)

**Example:**
```bash
bazel run //drake-ur-driver:ur_driver -- robot_ip_address=192.168.1.100 control_mode=position lcm_command_channel=UR_CMD
```

### Simulation

- https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/ursim_docker.html

### References

- https://github.com/RobotLocomotion/drake-franka-driver
- https://github.com/RobotLocomotion/drake-iiwa-driver
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
- https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver
- https://github.com/UniversalRobots/Universal_Robots_Client_Library
- https://github.com/ros-industrial/universal_robot
