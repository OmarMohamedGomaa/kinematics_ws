import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

from car_kinematics.robot_kinematics import (
    FourWheelOmniKinematics,
    DiffDriveKinematics,
    ThreeWheelOmniKinematics,
    MecanumKinematics
)

def pose_integration(vx, vy, wz, x, y, theta, dt):
    """
    Integrates computed chassis velocities over dynamic time
    steps (dt) to continuously update global pose coordinates (X, Y,
    theta).

    Parameters:
    vx : x component of the velocity
    vy : y component of the velocity
    wz : angular velocity around the z-axis
    x : current x position
    y : current y position
    theta : current angle of rotation in radians
    dt : time step for integration

    """
    X_dot = vx * np.cos(theta) - vy * np.sin(theta)
    Y_dot = vx * np.sin(theta) + vy * np.cos(theta)

    x += X_dot * dt
    y += Y_dot * dt
    theta += wz * dt

    return x, y, theta

class WheelOdometryNode(Node):
    # ROS 2 node for processing wheel odometry and broadcasting TF frames

    def __init__(self):
        super().__init__('odometry_node')

        self.declare_parameter('encoder_speeds_topic', '/encoder_speeds')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('wheel_base', 0.5)
        self.declare_parameter('drive_type', 'macnum')
        self.declare_parameter('track_width', 1.0)
        self.declare_parameter('wheel_radius', 0.5)

        encoder_speeds_topic = str(self.get_parameter('encoder_speeds_topic').value)
        odom_topic = str(self.get_parameter('odom_topic').value)
        wheel_base = self.get_parameter('wheel_base').value
        drive_type = self.get_parameter('drive_type').value
        track_width = self.get_parameter('track_width').value
        wheel_radius = self.get_parameter('wheel_radius').value

        if drive_type == "fourWheel":
            self.drive_kinematics = FourWheelOmniKinematics(L=wheel_base, W=track_width, R=wheel_radius)
        elif drive_type == "diffDrive":
            self.drive_kinematics = DiffDriveKinematics(L=wheel_base, W=track_width, R=wheel_radius)
        elif drive_type == "threeWheel":
            self.drive_kinematics = ThreeWheelOmniKinematics(L=wheel_base, W=track_width, R=wheel_radius)
        elif drive_type == "macnum":
            self.drive_kinematics = MecanumKinematics(L=wheel_base, W=track_width, R=wheel_radius)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # TF broadcaster setup
        self.tf_broadcaster = TransformBroadcaster(self)

        # publish
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)

        # feedback subscription
        self.subscribe_encoder = self.create_subscription(
            Float64MultiArray,
            encoder_speeds_topic,
            self.encoder_callback,
            10
        )

    def encoder_callback(self, msg: Float64MultiArray):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        if dt <= 0:
            self.last_time = current_time
            return

        # 2. Forward Kinematics
        w_measured = list(msg.data)
        vx, vy, wz = self.drive_kinematics.forward(w_measured)

        
        # 3. Pose Integration
        
        self.x , self.y, self.theta = pose_integration(vx, vy, wz, self.x, self.y, self.theta, dt)


        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        
       
        # 4. Telemetry Publication
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Pose in global frame
        odom_msg.pose.pose.position.x = float(self.x)
        odom_msg.pose.pose.position.y = float(self.y)
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = float(qz)
        odom_msg.pose.pose.orientation.w = float(qw)

        # Velocities in chassis/child frame
        odom_msg.twist.twist.linear.x = float(vx)
        odom_msg.twist.twist.linear.y = float(vy)
        odom_msg.twist.twist.angular.z = float(wz)

        self.odom_pub.publish(odom_msg)

        # 5. TF Broadcast (odom -> base_link)
        

        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = float(qz)
        t.transform.rotation.w = float(qw)

        self.tf_broadcaster.sendTransform(t)

        self.last_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()