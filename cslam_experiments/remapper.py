#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan, CameraInfo, Image
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from laser_geometry.laser_geometry import LaserProjection


class Remapper(Node):
    def __init__(self):
        super().__init__("remapper_odom")
        self.get_logger().info("Creating publishers")
        self._pub_odom = self.create_publisher(Odometry, "/odom_out", 10)
        self._pub_pointcloud = self.create_publisher(PointCloud2, "/pointcloud_out", 10)

        self._pub_color_image = self.create_publisher(Image, "/color/image_raw", 10)
        self._pub_depth_image = self.create_publisher(Image, "/aligned_depth_to_color/image_raw", 10)
        self._pub_color_info = self.create_publisher(CameraInfo, "/color/camera_info", 10)
        self._pub_depth_info = self.create_publisher(CameraInfo, "/aligned_depth_to_color/camera_info", 10)

        self.create_subscription(CameraInfo, "/camera/color/camera_info", self._camera_info_callback, 1)

        tss_rgbd = ApproximateTimeSynchronizer([
            Subscriber(self, Image, "/camera/color/image_raw"),
            Subscriber(self, Image, "/camera/depth/image_raw"),
            Subscriber(self, Odometry, "/odom_in")
        ],
            100,
            0.1
        )
        tss_rgbd.registerCallback(self._callback_rgbd)

        # tss = ApproximateTimeSynchronizer([
        #     Subscriber(self, LaserScan, "/scan_in"),
        #     Subscriber(self, Odometry, "/odom_in")
        # ],
        #     100,
        #     0.1
        # )

        tss = ApproximateTimeSynchronizer([
            Subscriber(self, PointCloud2, "/scan_in"),
            Subscriber(self, Odometry, "/odom_in")
        ],
            100,
            0.1
        )
        tss.registerCallback(self._callback_lidar)

        # self.projector = LaserProjection()

    def _callback_lidar(self, ls_msg, odom_msg):
        # pc_msg = self.projector.projectLaser(ls_msg)
        pc_msg = ls_msg

        self._pub_odom.publish(odom_msg)
        self._pub_pointcloud.publish(pc_msg)

    def _camera_info_callback(self, camera_info):
        self.camera_info = camera_info

    def _callback_rgbd(self, color_image, depth_image, odom_msg):
        if self.camera_info is not None:
            self.camera_info.header.stamp = color_image.header.stamp
            self._pub_odom.publish(odom_msg)
            self._pub_color_image.publish(color_image)
            self._pub_depth_image.publish(depth_image)
            self._pub_color_info.publish(self.camera_info)
            self._pub_depth_info.publish(self.camera_info)


if __name__ == '__main__':
    rclpy.init()
    remapper = Remapper()
    remapper.get_logger().info("Initialization Done.")
    rclpy.spin(remapper)
    rclpy.shutdown()
