import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import yaml
import os
import sys
from ament_index_python.packages import get_package_share_directory

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)

    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/GeoPose objects from the yaml file
        """
        gepose_wps = []
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
      

class WayPointsPublisher(Node):
    def __init__(self, yaml_file_path):
        super().__init__('way_points_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer_ = self.create_timer(1.0, self.publish_way_points)
        self.get_logger().info('WayPointsPublisher node has been initialized')

    def publish_way_points(self):
        way_point = PoseStamped()

        # Read waypoints from YAML file
        with open('waypoints.yaml', 'r') as file:
            waypoints = yaml.safe_load(file)

        # Set the desired waypoint coordinates
        way_point.pose.position.x = waypoints['x']
        way_point.pose.position.y = waypoints['y']
        way_point.pose.position.z = waypoints['z']
        way_point.pose.orientation.x = waypoints['qx']
        way_point.pose.orientation.y = waypoints['qy']
        way_point.pose.orientation.z = waypoints['qz']
        way_point.pose.orientation.w = waypoints['qw']

        self.publisher_.publish(way_point)
        self.get_logger().info('Published a waypoint')

def main(args=None):
    rclpy.init(args=args)

    default_yaml_file_path = os.path.join(get_package_share_directory(
        "nav2_gps_waypoint_follower_demo"), "config", "gps_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    way_points_publisher = WayPointsPublisher(yaml_file_path)
    rclpy.spin(way_points_publisher)
    way_points_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
