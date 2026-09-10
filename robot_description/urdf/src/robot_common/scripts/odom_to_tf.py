#!/usr/bin/env python3
"""
1. 订阅原始 /odom
2. 转发到 /dlio/odom_node/odom
3. 广播与消息一致的 odom -> base TF
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Gazebo P3D raw odometry
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Navigation odometry
        self.odom_pub = self.create_publisher(Odometry, '/dlio/odom_node/odom', 10)

        self.get_logger().info('Odometry relay started: /odom -> /dlio/odom_node/odom')

    def odom_callback(self, msg: Odometry):
        """Relay odometry and publish its matching TF"""
        self.odom_pub.publish(msg)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
