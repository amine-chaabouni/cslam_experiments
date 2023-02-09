import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from laser_geometry.laser_geometry import LaserProjection

class Remapper(Node):
    def __init__(self):
        super().__init__("remapper_odom")
        self._pub_odom = self.create_publisher(Odometry, "/odom_out", 10)
        self._pub_pointcloud = self.create_publisher(PointCloud2, "/pointcloud_out", 10)

        #tss = ApproximateTimeSynchronizer([
        #    Subscriber(self, LaserScan, "/scan_in"),
        #    Subscriber(self, Odometry, "/odom_in")
        #    ],
        #    100,
        #    0.1
        #)

        tss = ApproximateTimeSynchronizer([
            Subscriber(self, PointCloud2, "/scan_in"),
            Subscriber(self, Odometry, "/odom_in")
            ],
            100,
            0.1
        )
        tss.registerCallback(self._callback_lidar)

        self.projector = LaserProjection(self)


    def _callback_lidar(self, ls_msg, odom_msg):
        # pc_msg = self.projector.projectLaser(ls_msg)
        pc_msg = ls_msg

        self._pub_odom.publish(odom_msg)
        self._pub_pointcloud.publish(pc_msg)




def main(args=None):
    rclpy.init()
    remapper = Remapper()
    remapper.get_logger().info("Initialization Done.")
    rclpy.spin(remapper)