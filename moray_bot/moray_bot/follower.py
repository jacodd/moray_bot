import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator

import yaml
from ament_index_python.packages import get_package_share_directory

import os
import sys
import time

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_geometry_msgs import do_transform_pose, PoseStamped, Pose
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import MarkerArray, Marker

from moray_bot.utils import LLtoUTM, toPoseStamped


class FrameListener(Node):
    def __init__(self):
        super().__init__("frame_listener")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.subscriber = self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.subscriber
        self.gps = None

    def gps_callback(self, msg):
        self.gps = msg
        print("gps callback")

    def get_gps(self):
        gps = self.gps
        if gps is None:
            return None
        return (gps.latitude, gps.longitude)


    def get_transform(self):
        try:
            while not self.buffer.can_transform("utm", "map", rclpy.time.Time()):
                self.get_logger().info("Waiting for transform from map to utm")
                time.sleep(0.1)
            t = self.buffer.lookup_transform('utm', 'map', rclpy.time.Time())

        except Exception as e:
            print(e)
            print("Failed to get transform from map to utm")
            return None
        return t

class YamlWaypointParser:
    """
    Parse a set of gps waypoints from a yaml file
    """

    def __init__(self, wps_file_path: str) -> None:
        

        self.first = None
        self.firstE = None
        self.firstN = None
        self.firstYaw = None

        #########################################
        # self.frameListerner = FrameListener()
        # self.t = self.frameListerner.get_transform()
        # while self.frameListerner.get_gps() is None:
        #     time.sleep(0.1)
        # lat, long = self.frameListerner.get_gps()
        # self.firstE, self.firstN = LLtoUTM(lat, long)
        # self.firstYaw = 0.0
        #########################################


        with open(wps_file_path, 'r') as wps_file:
            self.wps_dict = yaml.safe_load(wps_file)
            
    def get_wps(self):
        """
        Get an array of geographic_msgs/msg/PoseStamped objects from the yaml file
        """
        poses_wps = []
        rclpy.logging.get_logger("yaml_parser").info("Received {} waypoints".format(len(self.wps_dict["waypoints"])-1))
        i = 1
        for wp in self.wps_dict["waypoints"]:
            latitude, longitude, yaw = wp["latitude"], wp["longitude"], wp["yaw"]
            east, north = LLtoUTM(latitude, longitude)

            #########################################
            if self.first is None:
                self.first = 1
                self.firstE = east
                self.firstN = north
                self.firstYaw = yaw
            else:
                rclpy.logging.get_logger("yaml_parser").info(
                "Wp {}: {}, {}, {}".format(i, latitude, longitude, yaw))
                i += 1
                poses_wps.append(toPoseStamped(east- self.firstE, north - self.firstN, yaw-self.firstYaw))
            #########################################
            # wp_pose = toPoseStamped(east, north, yaw)
            # wp_pose_t = PoseStamped()
            # wp_pose_t.pose = do_transform_pose(wp_pose.pose, self.t)
            # wp_pose_t.header.frame_id = "map"
            # poses_wps.append(wp_pose_t)
            #########################################

        return poses_wps

class wayPointPublisher(Node):
    def __init__(self):
        super().__init__("waypoint_publisher")
        self.publisher_ = self.create_publisher(MarkerArray, '/waypoints', 10)
    def publish(self, wps):
        msg = MarkerArray()

        for i, wp in enumerate(wps):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoint"
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose = wp.pose
            marker.scale.x = 0.33
            marker.scale.y = 0.33
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.2
            msg.markers.append(marker)
        self.publisher_.publish(msg)
        

class GpsWpCommander():
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self, wps_file_path):
        self.navigator = BasicNavigator("basic_navigator")
        self.wp_parser = YamlWaypointParser(wps_file_path)
        self.wpPublisher = wayPointPublisher()

    def start_wpf(self):
        """
        Function to start the waypoint following
        """
        # self.navigator.waitUntilNav2Active(localizer='robot_localization')
        wps = self.wp_parser.get_wps()

        self.wpPublisher.publish(wps)
        self.navigator.followWaypoints(wps)
        while (not self.navigator.isTaskComplete()):
            time.sleep(0.1)
        print("wps completed successfully")


def main():
    rclpy.init()

    # allow to pass the waypoints file as an argument
    default_yaml_file_path = os.path.join(get_package_share_directory(
        "moray_bot"), "config", "gnss_waypoints.yaml")
    if len(sys.argv) > 1:
        yaml_file_path = sys.argv[1]
    else:
        yaml_file_path = default_yaml_file_path

    


    gps_wpf = GpsWpCommander(yaml_file_path)
    gps_wpf.start_wpf()


if __name__ == "__main__":
    main()
