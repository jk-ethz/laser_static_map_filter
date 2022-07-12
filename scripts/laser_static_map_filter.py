#!/usr/bin/env python2
import rospy
import tf2_geometry_msgs.tf2_geometry_msgs
import tf2_ros
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PointStamped
from occupancy_grid_python import OccupancyGridManager

class LaserStaticMapFilter:
    def __init__(self):
        self._ogm = OccupancyGridManager('/map', subscribe_to_updates=True)
        self._scan_sub = rospy.Subscriber("/scan", PointCloud, self._scan_callback, queue_size=1)
        self._filtered_pub = rospy.Publisher("/scan_filtered", PointCloud, queue_size=1)

        self._map = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._point_tmp = PointStamped()

    def _filter(self, point, header):
        self._point_tmp.header = header
        self._point_tmp.point = point
        ogm_point = self._tf_buffer.transform(self._point_tmp, self._ogm.reference_frame)
        cost = self._ogm.get_cost_from_world_x_y(ogm_point.point.x, ogm_point.point.y)
        return cost >= 0 and cost < 100

    def _scan_callback(self, scan):
        if self._ogm is None:
            rospy.logwarn_throttle(3.0, "Did not receive map yet, cannot process scan.")
            return

        scan.points = [point for point in scan.points if self._filter(point, scan.header)]
        self._filtered_pub.publish(scan)


if __name__ == '__main__':
    rospy.init_node('laser_static_map_filter', anonymous=True)
    laser_static_map_filter = LaserStaticMapFilter()
    rospy.spin()
