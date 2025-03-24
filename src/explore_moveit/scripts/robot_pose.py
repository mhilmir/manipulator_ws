#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK
from std_msgs.msg import Header

class JointStateToPoseNode(Node):
    def __init__(self):
        super().__init__('joint_states_to_pose')
        
        # Create a client for the compute_fk service
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        while not self.fk_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /compute_fk service...')
        
        # Subscriber to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for the end-effector pose
        self.pose_publisher = self.create_publisher(PoseStamped, '/end_effector_pose', 10)
        
    def joint_state_callback(self, msg):
        # self.get_logger().info(f'Received Joint State: {msg.name}')
        
        # Prepare FK request
        request = GetPositionFK.Request()
        request.header = Header()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = ""  # Adjust frame as needed
        request.robot_state.joint_state = msg
        request.fk_link_names = ["end_effector_link"]  # Change to correct end-effector link
        
        # Call FK service
        future = self.fk_client.call_async(request)
        future.add_done_callback(self.fk_response_callback)
    
    def fk_response_callback(self, future):
        try:
            response = future.result()
            if response and response.pose_stamped:
                pose_msg = response.pose_stamped[0]
                self.pose_publisher.publish(pose_msg)
                self.get_logger().info(f'Published End Effector Pose: {pose_msg}')
            else:
                self.get_logger().warn('FK service returned no pose. Check joint names and FK link.')
        except Exception as e:
            self.get_logger().error(f'FK service call failed: {e}')


def main():
    rclpy.init()
    node = JointStateToPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1742829478, nanosec=379095373), frame_id=''),
# name=['joint2', 'joint3', 'joint1', 'joint4', 'gripper_left_joint', 'gripper_right_joint'],
# position=[-0.998621491772461, 0.6998369557401075, 0.0, 0.2992723823888618, -0.0099005171480255, 0.0],
# velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
# effort=[-19.0, -59.0, -2.0, -47.0, 0.0, 0.0]
# )