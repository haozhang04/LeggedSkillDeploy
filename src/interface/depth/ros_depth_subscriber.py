import threading

import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class ROSDepthSubscriber(Node):
    """Receive the latest normalized policy depth image in a ROS2 thread."""

    def __init__(self, depth):
        if not rclpy.ok():
            rclpy.init()
        super().__init__("go1_depth_subscriber")
        self._shutdown = False

        self.depth = depth
        self.bridge = CvBridge()
        sensor_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=5)
        self.subscription = self.create_subscription(Image, "/camera/depth/network_input", self.depth_callback, sensor_qos)
        self._spin_thread = threading.Thread(target=self._spin, name="ros-depth", daemon=True)
        self._spin_thread.start()

    def _spin(self):
        while rclpy.ok() and not self._shutdown:
            rclpy.spin_once(self, timeout_sec=0.1)

    def depth_callback(self, message):
        if message.encoding != "32FC1":
            self.get_logger().error(f"Unsupported depth encoding: {message.encoding}")
            return
        if (message.width, message.height) != (self.depth.width, self.depth.height):
            self.get_logger().error(f"Unexpected depth size: {message.width}x{message.height}")
            return

        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="32FC1").copy()
        self.depth.image = torch.from_numpy(image).unsqueeze(0).unsqueeze(0)

    def close(self):
        self._shutdown = True
        self._spin_thread.join(timeout=1.0)
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
