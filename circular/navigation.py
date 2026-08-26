import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import visualization_msgs.msg
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np

import circular.helper as helper
from circular.rrt_star import RRTStar

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        self.robot_pose_subscriber = self.create_subscription(
            msg_type=geometry_msgs.msg.Pose2D,
            topic='/pose',
            callback=self.read_robot_pose,
            qos_profile=10
        )

        self.return_path_subscriber = self.create_subscription(
            msg_type=geometry_msgs.msg.PoseArray,
            topic='/returned_path',
            callback=self.receive_path_action,
            qos_profile=10
        )

        self.robot_command_publisher = self.create_publisher(
            msg_type=geometry_msgs.msg.Twist,
            topic='/cmd_vel',
            qos_profile=10
        )

        self.robot_command_publisher_timer = self.create_timer(
            timer_period_sec=0.2,
            callback=self.move_robot
        )

        marker_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.delete_old_path_publisher = self.create_publisher(
            msg_type=visualization_msgs.msg.MarkerArray,
            topic='/sim_markers',
            qos_profile=marker_qos
        )
    

    def read_robot_pose(self, msg):
        if not hasattr(self, 'robot_orien'):
            self.get_logger().info(f"{msg.theta}")

        self.robot_pose = np.array([msg.x, msg.y])
        self.robot_orien = msg.theta


    def move_robot(self):   
        if not hasattr(self, 'path') or self.i >= len(self.path):
            self.publish_to_stop()
            return

        target = self.path[self.i]
        distance = -1
        theta_to_rotate = helper.normalize_angle(target[0] - helper.normalize_angle(self.robot_orien))
        if target[-1] is not None:          
            distance = np.linalg.norm(self.robot_pose - target)
            theta_to_rotate = helper.normalize_angle(helper.get_angle(self.robot_orien, self.robot_pose, target))

        msg = geometry_msgs.msg.Twist()

        if 0 <= distance <= helper.DIST_TOL:
            self.i += 1
            return
        
        # Rotate first if needed
        if abs(theta_to_rotate) > helper.ANGULAR_TOL:
            msg.linear.x = 0.0
            msg.angular.z = np.sign(theta_to_rotate) * min(
                helper.K_a * abs(theta_to_rotate),
                helper.ANGULAR_VELOCITY
            )
            self.robot_command_publisher.publish(msg)
            return
        
        # Move toward target segment when the orientation aligned
        if distance > helper.DIST_TOL:
            msg.angular.z = 0.0
            msg.linear.x = min(helper.K_v * distance, helper.LINEAR_VELOCITY)

        self.robot_command_publisher.publish(msg)
        

    def receive_path_action(self, msg: geometry_msgs.msg.PoseArray):
        # Delete old path
        self.delete_old_path()
        self.i = 0
        self.path = np.array([[p.position.x, p.position.y] if p.position.z == 0.0
                              else [p.orientation.w, None] for p in msg.poses])


    def publish_to_stop(self):
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.robot_command_publisher.publish(msg)


    def delete_old_path(self):
        marker_array = visualization_msgs.msg.MarkerArray()
        marker = visualization_msgs.msg.Marker()
        # marker.id = -2
        marker.ns = 'trace'
        marker.action = visualization_msgs.msg.Marker.DELETEALL
        marker.header.frame_id = 'world'
        marker_array.markers.append(marker)
        self.delete_old_path_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()