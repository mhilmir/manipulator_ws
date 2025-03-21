import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander

rclpy.init()
node = Node("get_robot_pose")
arm_group = MoveGroupCommander("arm")  # Use your planning group name

# Get current end-effector pose
current_pose: Pose = arm_group.get_current_pose().pose
print("Position:", current_pose.position)
print("Orientation (quaternion):", current_pose.orientation)

rclpy.shutdown()
