import rclpy
from rclpy.node import Node
import visualization_msgs.msg
import std_msgs.msg

import circular.helper as helper

from std_srvs.srv import Empty

class DrawObstacles(Node):
    def __init__(self):
        super().__init__('draw_obstacles')

        # Create client to request Empty to reset whenever start the controller
        self.client = self.create_client(
            srv_type=Empty,
            srv_name="/reset",
        )
        self.request = Empty.Request()


        obstacles = helper.get_obstacles(self)
        obstacles[:, -1] -= helper.RADIUS

        self.obst_marker_publisher = self.create_publisher(
            msg_type=visualization_msgs.msg.MarkerArray,
            topic='/obstacles',
            qos_profile=10
        )


        self.publish_obstacles(obstacles)
    

    def publish_obstacles(self, obstacles):
        markers = visualization_msgs.msg.MarkerArray()
        for i, (x, y, r) in enumerate(obstacles):
            marker = visualization_msgs.msg.Marker()
            marker.header.frame_id = 'world'
            marker.id = i + 1000
            marker.type = visualization_msgs.msg.Marker.CYLINDER
            marker.action = visualization_msgs.msg.Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0

            marker.scale.x = 2 * r
            marker.scale.y = 2 * r
            marker.scale.z = 0.5

            marker.color = std_msgs.msg.ColorRGBA(r=1.0, g=0.5, b=0.5, a=1.0)

            markers.markers.append(marker)
        
        self.obst_marker_publisher.publish(markers)


    def send_request(self):
        return self.client.call_async(self.request)
    

def main(args=None):
    rclpy.init(args=args)
    draw_obstacles = DrawObstacles()

    future = draw_obstacles.send_request()
    rclpy.spin_until_future_complete(draw_obstacles, future)

    rclpy.spin(draw_obstacles)
    draw_obstacles.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()