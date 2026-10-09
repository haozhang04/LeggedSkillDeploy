# Copyright (c) 2024-2025 zh
"""订阅并显示 go1_visual_embedding.py 发布的深度图。

用法：
    python go1_view_realsense_depth.py

按 r 开始或停止录制网络输入，按 q、Esc 或 Ctrl+C 退出。
"""

import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image


class DepthViewer(Node):

    def __init__(self):
        super().__init__("go1_depth_viewer")
        self.bridge = CvBridge()
        self.raw_frame = None
        self.network_frame = None
        self.network_frame_id = 0

        sensor_qos = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.raw_subscription = self.create_subscription(Image, "/camera/depth/image_raw", self._raw_callback, sensor_qos)
        self.network_subscription = self.create_subscription(Image, "/camera/depth/network_input", self._network_callback, sensor_qos)
        self.get_logger().info("Start viewer, press 'r' to start or stop recording")

    def _raw_callback(self, message):
        if message.encoding not in ("16UC1", "mono16"):
            self.get_logger().warning(f"Unsupported raw depth encoding: {message.encoding}", throttle_duration_sec=2.0)
            return
        try:
            raw = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            # go1_visual_embedding.py 的 far_clip 为 2 m，RealSense Z16 单位通常为毫米。
            self.raw_frame = np.clip(raw.astype(np.float32) / 2000.0, 0.0, 1.0)
        except CvBridgeError as error:
            self.get_logger().error(f"Failed to decode raw depth: {error}")

    def _network_callback(self, message):
        if message.encoding != "32FC1":
            self.get_logger().warning(f"Unsupported network input encoding: {message.encoding}", throttle_duration_sec=2.0)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="32FC1")
            self.network_frame = np.clip(image.astype(np.float32) + 0.5, 0.0, 1.0)
            self.network_frame_id += 1
        except CvBridgeError as error:
            self.get_logger().error(f"Failed to decode network input: {error}")


def main():
    rclpy.init()
    node = DepthViewer()
    raw_window = "RealSense raw depth (ROS 2)"
    network_window = "Network depth observation (ROS 2)"
    cv2.namedWindow(raw_window, cv2.WINDOW_NORMAL)
    cv2.namedWindow(network_window, cv2.WINDOW_NORMAL)
    writer = None
    recorded_frame_id = 0
    output = None

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            if node.raw_frame is not None:
                cv2.imshow(raw_window, node.raw_frame)
            if node.network_frame is not None:
                cv2.imshow(network_window, node.network_frame)
                if writer is not None and node.network_frame_id != recorded_frame_id:
                    frame = (node.network_frame * 255).astype(np.uint8)
                    writer.write(cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR))
                    recorded_frame_id = node.network_frame_id
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                if writer is not None:
                    writer.release()
                    writer = None
                    print(f"saved {output}")
                elif node.network_frame is None:
                    print("No network depth frame received yet")
                else:
                    output = Path(f"recordings/real_depth_{time.strftime('%Y%m%d_%H%M%S')}.mp4")
                    output.parent.mkdir(parents=True, exist_ok=True)
                    height, width = node.network_frame.shape
                    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (width, height))
                    if not writer.isOpened():
                        writer.release()
                        writer = None
                        raise RuntimeError(f"Failed to open video file: {output}")
                    recorded_frame_id = node.network_frame_id
                    print(f"recording {output}")
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
            print(f"saved {output}")
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
