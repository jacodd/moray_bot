import math
from pyproj import Proj, transform
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


def euler_from_quaternion(q: Quaternion):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z


def latLonYaw2Geopose(latitude: float, longitude: float, yaw: float = 0.0) -> GeoPose:
    """
    Creates a geographic_msgs/msg/GeoPose object from latitude, longitude and yaw
    """
    geopose = GeoPose()
    geopose.position.latitude = latitude
    geopose.position.longitude = longitude
    geopose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return geopose

def latLon2poseStamped(latitude: float, longitude: float, yaw: float = 0.0) -> PoseStamped:
    """
    Creates a geometry_msgs/msg/PoseStamped object from latitude, longitude
    """
    poseStamped = PoseStamped()
    poseStamped.header.frame_id = "wgs84"
    poseStamped.pose.position.x = latitude
    poseStamped.pose.position.y = longitude
    poseStamped.pose.orientation = quaternion_from_euler(0, 0, yaw)
    return poseStamped

grid_size = 100000.0  # 100 km grid

def LLtoUTM(lat, lon):
    utm_proj = Proj(proj='utm', zone=math.floor((lon + 180) / 6) + 1, ellps='WGS84')
    easting, northing = utm_proj(lon, lat)
    return easting, northing

def UTMtoLL(UTMNorthing, UTMEasting, UTMZone):
    utm_proj = Proj(proj='utm', zone=UTMZone, ellps='WGS84')
    wgs84_proj = Proj(proj='latlong', datum='WGS84')
    lon, lat = transform(utm_proj, wgs84_proj, UTMEasting, UTMNorthing)
    return lat, lon

def toPoseStamped(latitude, longitude, yaw = 0.0):
    """
    Creates a geometry_msgs/msg/PoseStamped object from latitude, longitude
    """
    poseStamped = PoseStamped()
    poseStamped.header.frame_id = "map"
    poseStamped.pose.position.x = latitude
    poseStamped.pose.position.y = longitude
    poseStamped.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
    return poseStamped

