import numpy as np
import math
from rclpy.node import Node

RADIUS = 0.5
OFFSET = 0.01
EPS = 0.05
ANGULAR_VELOCITY = 1.0      # Robot's angular velocity
LINEAR_VELOCITY = 3.0       # Robot's linear velocity
DIST_TOL = 0.05             # Distance tolerance to stop the robot when it reaches desired point
ANGULAR_TOL = 0.001         # Angular tolerance to stop the robot when it reaches desired point
K_a = 2.0 
K_v = 2.0


def quaternion_to_yaw(z, w):
    return 2 * math.atan2(z, w)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_angle(theta_robot, point_robot, point):
    dy = point[1] - point_robot[1]
    dx = point[0] - point_robot[0]

    theta_point = np.arctan2(dy, dx)

    angle_error = theta_point - theta_robot

    angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

    return angle_error


def get_obstacles(node: Node):
    """
    "1.0,2.0,3.0; 0.5,1.0,1.0" = "x,y,r; x,y,r; ..."
    """
    node.declare_parameter('obstacles', '')
    obstacles_string = node.get_parameter('obstacles').value

    obstacles = []
    for obstacle in obstacles_string.split(';'):
        obstacle = obstacle.strip().split(',')
        if len(obstacle) != 3:
            raise ValueError(f"Invalid format {obstacle}")
        
        try:
            x, y, r = map(float, obstacle)
        except:
            raise ValueError(f"Invalide number value")

        obstacles.append([x, y, r])

    obstacles = np.array(obstacles)
    obstacles[:, -1] += RADIUS

    node.get_logger().info(f"Obstacles read: {obstacles}")
    return obstacles

