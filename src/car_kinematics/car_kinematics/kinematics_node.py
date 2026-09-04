
from robot_kinematics import FourWheelOmniKinematics , DiffDriveKinematics ,ThreeWheelOmniKinematics , MecanumKinematics
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class KinematicsNode(Node):
	"""Represent the car kinematics ROS 2 node."""

	def __init__(self):
		super().__init__('kinematics_node')
		self.declare_parameter('cmd_vel_topic', '/cmd_vel')
		self.declare_parameter('wheels_topic', '/wheel_set_points')
		self.declare_parameter('wheel_base',0.5)
		self.declare_parameter('drive_type',"macnum")
		self.declare_parameter('track_width',1.0)
		self.declare_parameter('wheel_radius',0.5)
			 
		cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
		wheels_topic = self.get_parameter('wheels_topic').value
		wheel_base = self.get_parameter('wheel_base').value
		drive_type = self.get_parameter('drive_type').value
		track_width = self.get_parameter('track_width').value
		wheel_radius = self.get_parameter('wheel_radius').value
        
		self.subscribe_vel = self.create_subscription(Twist, cmd_vel_topic,self.vel_callbacks,10)       
		self.pub_wheels = self.create_publisher(Float64MultiArray, wheels_topic, 10)  
		if drive_type == "fourWheel":
			self.drive_kinematics = FourWheelOmniKinematics(L=wheel_base , W=track_width, R=wheel_radius)
		elif drive_type == "diffDrive":
			self.drive_kinematics = DiffDriveKinematics(L=wheel_base , W=track_width, R=wheel_radius)
		elif drive_type == "threeWheel":
			self.drive_kinematics = ThreeWheelOmniKinematics(L=wheel_base , W=track_width, R=wheel_radius)
		elif drive_type == "macnum":
			self.drive_kinematics = MecanumKinematics(L=wheel_base , W=track_width, R=wheel_radius)
		
		
	def vel_callback(self,msg:Twist):
		vx,vy,wz = msg.linear.x , msg.linear.y , msg.angular.z
		valus = Float64MultiArray()
		res = self.drive_kinematics.inverse(vx,vy,wz)
		for val in res:
			valus.data.append(val)
		self.pub_wheels.publish(valus)
			
	

def main(args=None):
	rclpy.init(args=args)
	node = KinematicsNode('Mecanum', 0.5, 0.5, 0.1)

	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	finally:
		node.destroy_node()
		rclpy.shutdown()


if __name__ == '__main__':
	main()
