# car_kinematics

A ROS 2 package for mobile robot kinematics and wheel odometry. The package provides configurable inverse and forward kinematics models for four popular drive configurations, mapping velocity commands (`Twist`) to wheel angular speeds and calculating dead-reckoning odometry with TF frame broadcasts.

---

## Supported Drive Types

* **Mecanum Drive (`macnum`)**: 4-wheel Mecanum wheel setup capable of omnidirectional motion.
* **4-Wheel Omni (`fourWheel`)**: 4-wheel omnidirectional platform with wheels mounted at right angles.
* **3-Wheel Omni (`threeWheel`)**: 3-wheel omnidirectional layout arranged at equal angular spacing 120 degrees.
* **Differential Drive (`diffDrive`)**: Standard 2-wheel skid-steer differential platform.

---

## Nodes

### 1. Kinematics Node (`kinematics_node`)

Subscribes to linear and angular velocity commands and computes individual wheel angular velocity setpoints using inverse kinematics.

* **Subscribed Topics:**
* `cmd_vel` (`geometry_msgs/msg/Twist`): Target linear (vx, vy) and angular (wz) chassis velocity.


* **Published Topics:**
* `wheel_set_points` (`std_msgs/msg/Float64MultiArray`): Target angular velocities rad/s for each wheel.



### 2. Wheel Odometry Node (`wheel_odometry_node`)

Subscribes to encoder wheel speeds, calculates body velocity using forward kinematics, integrates the pose over time x, y, theta, and publishes TF transforms and odometry messages.

* **Subscribed Topics:**
* `encoder_speeds` (`std_msgs/msg/Float64MultiArray`): Measured wheel speeds from feedback or motor encoders.


* **Published Topics:**
* `odom` (`nav_msgs/msg/Odometry`): Estimated pose and velocity in the global frame.


* **TF Transforms Broadcasted:**
* `odom` --> `base_link`



---

## Parameters

Both nodes share common geometric and configuration parameters:

| Parameter | Type | Default | Description |
| --- | --- | --- | --- |
| `drive_type` | `string` | `"macnum"` | Drive type: `fourWheel`, `diffDrive`, `threeWheel`, `macnum`. |
| `wheel_base` | `double` | `0.5` | Longitudinal distance from chassis center to wheel axis ($L$, meters). |
| `track_width` | `double` | `1.0` | Transverse distance between left and right wheels ($W$, meters). |
| `wheel_radius` | `double` | `0.5` | Radius of the wheels ($R$, meters). |
| `cmd_vel_topic` | `string` | `"/cmd_vel"` | Topic name for velocity commands (`kinematics_node`). |
| `wheels_topic` | `string` | `"/wheel_set_points"` | Topic name for output wheel setpoints (`kinematics_node`). |
| `encoder_speeds_topic` | `string` | `"/encoder_speeds"` | Topic name for feedback wheel speeds (`wheel_odometry_node`). |
| `odom_topic` | `string` | `"/odom"` | Topic name for publishing odometry (`wheel_odometry_node`). |

---

## Installation & Build

1. **Clone the workspace:**
```bash
cd ~/ros2_ws
git clone https://github.com/OmarMohamedGomaa/kinematics_ws.git

```


2. **Build the package:**
```bash
colcon build --packages-select car_kinematics
source install/setup.bash
```

---

## Usage Example (Each in Sepearate Terminal)

### Running the Kinematics Node

```bash
ros2 run car_kinematics kinematics_node --ros-args \
  -p drive_type:=fourWheel \
  -p wheel_base:=0.8 \
  -p track_width:=1.2 \
  -p wheel_radius:=0.1 \
  -p cmd_vel_topic:=/cmd_vel \
  -p wheels_topic:=/wheel_set_points

```

### Running the Wheel Odometry Node

```bash
ros2 run car_kinematics wheel_odometry_node --ros-args \
  -p drive_type:=fourWheel \
  -p wheel_base:=0.8 \
  -p track_width:=1.2 \
  -p wheel_radius:=0.1 \
  -p encoder_speeds_topic:=/wheel_set_points \
  -p odom_topic:=/odom

```

### Testing with `teleop_twist_keyboard`

Publish `/cmd_vel` commands to test node responses:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

### See if it works 

```bash
ros2 topic echo /odom

```
