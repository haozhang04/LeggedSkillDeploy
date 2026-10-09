#!/usr/bin/env python3
# Copyright (c) 2024-2025 zh
"""采集并发布 RealSense 深度图及其网络输入。

用法：
    python go1_visual_embedding.py

发布原始深度图和预处理后的 64×64 网络输入，按 Ctrl+C 退出。
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

class RealSenseDepthPublisher(Node):
    def __init__(self):
        super().__init__("go1_realsense_depth")

        self.input_width = 480
        self.input_height = 270
        self.camera_fps = 30
        self.publish_fps = self.camera_fps
        self.output_size = (64, 64)
        self.near_clip = 0.0
        self.far_clip = 2.0
        self.crop_top = 1
        self.crop_bottom = 3
        self.crop_left = 105
        self.crop_right = 109
        self.frame_id = "go1_depth_camera"
        self.bridge = CvBridge()

        self.raw_publisher = self.create_publisher(Image, "/camera/depth/image_raw", 1)
        self.network_input_publisher = self.create_publisher(Image, "/camera/depth/network_input", 1)

        # RealSense 原始 Z16 深度
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.input_width, self.input_height, rs.format.z16, self.camera_fps)
        profile = self.pipeline.start(config)
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()

        # 空洞填补：补全无效深度像素
        self.hole_filling_filter = rs.hole_filling_filter(2)
        # 空间滤波：降低单帧空间噪声
        self.spatial_filter = rs.spatial_filter()
        self.spatial_filter.set_option(rs.option.filter_magnitude, 5)
        self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.75)
        self.spatial_filter.set_option(rs.option.filter_smooth_delta, 1)
        self.spatial_filter.set_option(rs.option.holes_fill, 4)
        # 时间滤波：降低连续帧抖动
        self.temporal_filter = rs.temporal_filter()
        self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.75)
        self.temporal_filter.set_option(rs.option.filter_smooth_delta, 1)

        self.timer = self.create_timer(1.0 / self.publish_fps, self.publish_depth)
        self.get_logger().info(f"Publishing depth topics at {self.publish_fps} Hz")

    def publish_depth(self):
        try:
            frames = self.pipeline.wait_for_frames(1000)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                return

            depth_frame = self.hole_filling_filter.process(depth_frame)
            # depth_frame = self.spatial_filter.process(depth_frame)
            # depth_frame = self.temporal_filter.process(depth_frame)
            raw_depth = np.asanyarray(depth_frame.get_data())

            depth_m = raw_depth.astype(np.float32) * self.depth_scale
            depth_m = depth_m[self.crop_top:self.input_height - self.crop_bottom, self.crop_left:self.input_width - self.crop_right]
            depth_m = cv2.resize(depth_m, self.output_size, interpolation=cv2.INTER_NEAREST)
            valid = np.isfinite(depth_m) & (depth_m > self.near_clip)
            depth_m = np.where(valid, depth_m, self.far_clip)
            depth_m = np.clip(depth_m, self.near_clip, self.far_clip)
            network_input = (depth_m / (self.far_clip - self.near_clip) - 0.5).astype(np.float32)

            stamp = self.get_clock().now().to_msg()
            raw_message = self.bridge.cv2_to_imgmsg(raw_depth, encoding="16UC1")
            raw_message.header.stamp = stamp
            raw_message.header.frame_id = self.frame_id
            self.raw_publisher.publish(raw_message)

            network_input_message = self.bridge.cv2_to_imgmsg(network_input, encoding="32FC1")
            network_input_message.header.stamp = stamp
            network_input_message.header.frame_id = self.frame_id
            self.network_input_publisher.publish(network_input_message)
        except RuntimeError as error:
            self.get_logger().warning(f"RealSense frame error: {error}")

    def close(self):
        self.pipeline.stop()


def main():
    rclpy.init()
    node = RealSenseDepthPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
