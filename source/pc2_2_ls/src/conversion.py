#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from sensor_msgs.msg import PointCloud2, LaserScan

NUM_RANGES = 1875  # Number of ranges in a full scan
SCAN_ANGLE = np.pi * 2.0  # Full scan angle in radians
SCAN_RES = SCAN_ANGLE / NUM_RANGES  # Angular resolution of each range in radians
SCAN_RANGE = 100.0  # Maximum range of the laser scanner

TARGET_HEIGHT_MIN = -1e-2
TARGET_HEIGHT_MAX = 1e-2

def callback(data):
    # Convert the point cloud message to a generator of (x, y, z, intensity) tuples
    gen = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)

    # Extract the (x, y, z) tuples from the generator and put them into a numpy array
    points = np.array([p for p in gen])

    # Filter out NaN and Inf values in z-coordinate.
    z_values = points[:, 2]
    mask = np.logical_and(np.isfinite(z_values), np.logical_and(z_values > TARGET_HEIGHT_MIN, z_values < TARGET_HEIGHT_MAX))
    points = points[mask]

    # Convert points to polar coordinates (r, theta)
    ranges = np.linalg.norm(points[:, :2], axis=1)
    angles = np.arctan2(points[:, 1], points[:, 0])

    # Create an array of NaN values
    ranges_nan = np.empty(NUM_RANGES)
    ranges_nan[:] = np.nan

    # Fill the ranges array with the corresponding ranges
    for r, theta in zip(ranges, angles):
        index = int((theta + SCAN_ANGLE / 2) / SCAN_RES)  # Calculate the index of the range
        if 0 <= index < NUM_RANGES:
            ranges_nan[index] = r

    # Create a LaserScan message
    scan_msg = LaserScan()
    scan_msg.header = data.header
    scan_msg.angle_min = -SCAN_ANGLE / 2
    scan_msg.angle_max = SCAN_ANGLE / 2
    scan_msg.angle_increment = SCAN_RES
    scan_msg.range_min = 0.0
    scan_msg.range_max = SCAN_RANGE
    scan_msg.ranges = ranges_nan.tolist()

    # Publish the LaserScan message
    scan_publisher.publish(scan_msg)

if __name__ == '__main__':
    rospy.init_node('point_cloud2_to_laserscan', anonymous=True)
    scan_publisher = rospy.Publisher('/scan', LaserScan, queue_size=10)
    rospy.Subscriber('/velodyne1_points', PointCloud2, callback)
    rospy.spin()

