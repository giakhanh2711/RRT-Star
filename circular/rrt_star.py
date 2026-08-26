import rclpy
from rclpy.node import Node
import geometry_msgs.msg
import visualization_msgs.msg
from std_msgs.msg import ColorRGBA

import numpy as np
import math
import circular.helper as helper

class SampleNode:
    def __init__(self, position: np.ndarray, cost=0.0, parent=None):
        self.pose = position
        self.cost = cost
        self.parent = parent

    def __repr__(self):
        return f"{self.pose}"


class RRTStar(Node):
    def __init__(self, interval=1):
        super().__init__("rrt_star")
        self.obstacles = helper.get_obstacles(self)
        self.interval = interval

        self.bottom_left = np.min(self.obstacles[:, :-1], axis=0)
        self.top_right = np.max(self.obstacles[:, :-1], axis=0)

        self.goal_subscriber = self.create_subscription(
            msg_type=geometry_msgs.msg.PoseStamped,
            topic='/goal_pose',
            callback=self.read_goal,
            qos_profile=10,
        )

        self.robot_pose_subscriber = self.create_subscription(
            msg_type=geometry_msgs.msg.Pose2D,
            topic='/pose',
            callback=self.read_robot_pose,
            qos_profile=10
        )
        
        self.update_graph_publisher = self.create_publisher(
            msg_type=visualization_msgs.msg.MarkerArray,
            topic='/current_graph',
            qos_profile=10
        )

        self.path_publisher = self.create_publisher(
            msg_type=geometry_msgs.msg.PoseArray,
            topic='/returned_path',
            qos_profile=10
        )

        self.sample_nodes_marker = self.prepare_marker(type=visualization_msgs.msg.Marker.POINTS, 
                                                       action=visualization_msgs.msg.Marker.ADD,
                                                       color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0),
                                                       id=2)

        self.real_sample_nodes_marker = self.prepare_marker(type=visualization_msgs.msg.Marker.POINTS, 
                                                            action=visualization_msgs.msg.Marker.ADD,
                                                            color=ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0),
                                                            id=3)

    def prepare_marker(self, type, action, color: ColorRGBA, id):    
        marker = visualization_msgs.msg.Marker()
        marker.header.frame_id = 'world'
        marker.id = id
        marker.type = type
        marker.action = action
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color = color
        return marker


    def publish_marker_graph(self):
        marker_array = visualization_msgs.msg.MarkerArray()
        marker_array.markers.append(self.sample_nodes_marker)
        marker_array.markers.append(self.real_sample_nodes_marker)

        self.update_graph_publisher.publish(marker_array)

    def update_bounds(self, new_position):
        self.bottom_left = np.min(np.vstack((new_position, self.bottom_left)), axis=0)
        self.top_right = np.max(np.vstack((new_position, self.top_right)), axis=0)
    
    
    def read_robot_pose(self, msg):
        self.robot_pose = np.array([msg.x, msg.y])
        self.robot_orien = msg.theta

        self.update_bounds(self.robot_pose)


    def read_goal(self, msg):
        self.get_logger().info("Read goal called")
        self.graph = [SampleNode(self.robot_pose)]
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.goal_orien = msg.pose.orientation

        self.update_bounds(self.goal)

        marker_array = visualization_msgs.msg.MarkerArray()
        goal_marker = visualization_msgs.msg.Marker()
        goal_marker.points = []
        goal_marker.header.frame_id = 'world'
        goal_marker.id = 0
        goal_marker.type = visualization_msgs.msg.Marker.ARROW
        goal_marker.action = visualization_msgs.msg.Marker.ADD
        goal_marker.color.r = 0.5
        goal_marker.color.g = 1.0
        goal_marker.color.b = 0.1
        goal_marker.color.a = 1.0
        goal_marker.scale.x = 1.0
        goal_marker.scale.y = 0.1
        goal_marker.scale.z = 0.1
        goal_marker.pose.position.x = self.goal[0]
        goal_marker.pose.position.y = self.goal[1]
        goal_marker.pose.position.z = 0.0
        goal_marker.pose.orientation = self.goal_orien
        marker_array.markers.append(goal_marker)
        self.update_graph_publisher.publish(marker_array)
        
        path = self.planning()
        self.publish_path_msg(path)

    def publish_path_msg(self, path: np.ndarray):
        msg = geometry_msgs.msg.PoseArray()
        for pos in path:
            p = geometry_msgs.msg.Pose()
            if pos[-1] is None:
                p.orientation.w = pos[0]
                p.position.z = 1.0
            else:
                p.position.x = pos[0]
                p.position.y = pos[1]
                p.position.z = 0.0

            msg.poses.append(p)

        self.path_publisher.publish(msg)


    def check_collision_segment(self, from_pose: np.ndarray, to_pose: np.ndarray):
        """
        Return Pose
        """
        
        segment = to_pose - from_pose
        min_dist = np.linalg.norm(segment)
        segment_unit = segment / min_dist

        is_collide = False

        for obstacle in self.obstacles:
            center = obstacle[:-1]
            radius = obstacle[-1] + helper.EPS

            center_a = center - from_pose
            len_center_a = np.linalg.norm(center_a)
            from_to_perp = np.dot(center_a, segment_unit)

            distance_sq = len_center_a**2 - from_to_perp**2

            if distance_sq < radius**2:
                is_collide = True
                inside = np.sqrt(radius**2 - distance_sq)
                from_to_edge = from_to_perp - inside

                if -1e-9 <= from_to_edge < min_dist:
                    min_dist = from_to_edge

        return is_collide, self.get_coor(from_pose, to_pose, min_dist)
    

    def get_coor(self, from_pose, to_pose, dist):
        theta = math.atan2(to_pose[1] - from_pose[1], to_pose[0] - from_pose[0])
        return np.array([from_pose[0] + dist * math.cos(theta),
                from_pose[1] + dist * math.sin(theta)])
    

    def get_qnew(self, nearest_neighbor: SampleNode, qrand: SampleNode):
        """
        Return object SampleNode
        """
        dist = np.linalg.norm(qrand.pose - nearest_neighbor.pose)
        qnew_pose = qrand.pose

        # Steer
        if dist > self.interval:
            qnew_pose = self.get_coor(nearest_neighbor.pose, qnew_pose, self.interval)
            
        _, qnew_pose = self.check_collision_segment(nearest_neighbor.pose, qnew_pose)
        return SampleNode(qnew_pose)


    def extend(self, qrand: SampleNode) -> SampleNode:
        """
        Return nearest neighbor SampleNode
        """
        points = self.get_array()
        dist = np.linalg.norm(qrand.pose - points[:, :-1], axis=1)
        return self.graph[np.argmin(dist)]

       
    def get_neighbors(self, node):
        """
        Return a list of SampleNode neighbors to a node
        """
        points = self.get_array()
        dist = np.linalg.norm(node.pose - points[:, :-1], axis=1)
        mask = dist <= self.interval
        neighbors = []

        for i in np.arange(points.shape[0])[mask]:
            neighbor_node = self.graph[i]
            if np.array_equal(neighbor_node.pose, node.pose):
                continue
            is_collide, _ = self.check_collision_segment(node.pose, neighbor_node.pose)
            if not is_collide:
                neighbors.append(neighbor_node)

        return neighbors
    

    def get_array(self):
        """
        Return [N_centers x 3]
        """
        arrays = []
        for node in self.graph:
            arr = node.pose
            arr = np.append(arr, node.cost)
            arrays.append(arr)
        return np.array(arrays)
    

    def connect_with_parent(self, nearest_neibor: SampleNode, qnew: SampleNode):
        """
        Return qnew connected to the parent
        """
        neighbors = self.get_neighbors(qnew)
        min_dist = nearest_neibor.cost + np.linalg.norm(nearest_neibor.pose - qnew.pose)
        best_parent = nearest_neibor
        for node in neighbors:
            dist = node.cost + np.linalg.norm(node.pose - qnew.pose)
            if dist < min_dist:
                min_dist = dist
                best_parent = node

        qnew.cost = min_dist
        qnew.parent = best_parent


    def rewire_tree(self, qnew_node: SampleNode):
        neighbors = self.get_neighbors(qnew_node)
        for node in neighbors:
            if np.array_equal(qnew_node.parent.pose, node.pose):
                continue

            cost_to_check = qnew_node.cost + np.linalg.norm(qnew_node.pose - node.pose)
            if node.cost > cost_to_check:
                node.parent = qnew_node
                self.update_cost(node)

    def update_cost(self, node: SampleNode):
        node.cost = node.parent.cost + np.linalg.norm(node.pose - node.parent.pose)
        for n in self.graph:
            if n.parent == node:
                self.update_cost(n)

    def get_random_node(self, i) -> SampleNode:
        if i % 20 == 0:
            return SampleNode(self.goal)
        
        x = np.random.uniform(low=self.bottom_left[0], high=self.top_right[0])
        y = np.random.uniform(low=self.bottom_left[1], high=self.top_right[1])
        return SampleNode(np.array([x, y]))


    def prepare_point_to_draw(self, sample_node:SampleNode):
        p = geometry_msgs.msg.Point()
        p.x = sample_node.pose[0]
        p.y = sample_node.pose[1]
        p.z = 0.0
        return p

    def planning(self):
        i = 1
        while True:
            # Random node
            qrand = self.get_random_node(i)
            self.sample_nodes_marker.points.append(self.prepare_point_to_draw(qrand))

            # Extend
            nearest_neighbor = self.extend(qrand)

            if np.linalg.norm(nearest_neighbor.pose - qrand.pose) <= helper.EPS:
                continue

            qnew = self.get_qnew(nearest_neighbor, qrand)
            
            self.real_sample_nodes_marker.points.append(self.prepare_point_to_draw(qnew))
            self.publish_marker_graph()
            
            # Find best parent
            self.connect_with_parent(nearest_neighbor, qnew)

            # Rewire tree
            self.rewire_tree(qnew)
            self.graph.append(qnew)
            # self.get_logger().info(f"Debug qnew distance to obstacles: {self.obstacles[0][-1]}->{np.linalg.norm(qnew.pose - self.obstacles[0][:-1])} \
            #                        {self.obstacles[1][-1]}->{np.linalg.norm(qnew.pose - self.obstacles[1][:-1])}")

            if np.all(qnew.pose == self.goal):
                path = []
                self.get_path_to_node(qnew, path)
                path.append(np.array([helper.quaternion_to_yaw(self.goal_orien.z, self.goal_orien.w), None]))
                self.get_logger().info(f"path {path}")
                return path

            i += 1

    def get_path_to_node(self, node: SampleNode, path):
        if not node.parent:
            return

        self.get_path_to_node(node.parent, path)
        path.append(node.pose)


def main(args=None):
    rclpy.init(args=args)
    rrt_star = RRTStar(interval=1.0)
    rclpy.spin(rrt_star)
    rrt_star.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
