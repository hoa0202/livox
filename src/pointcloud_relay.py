#!/usr/bin/env python3
"""
Simple relay node that subscribes to a PointCloud2 topic,
changes the frame_id, and republishes to a new topic.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu


class PointCloudRelay(Node):
    def __init__(self):
        super().__init__('pointcloud_relay')
        
        # Declare parameters
        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('frame_id', '')
        self.declare_parameter('msg_type', 'pointcloud')  # 'pointcloud' or 'imu'
        
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        msg_type = self.get_parameter('msg_type').get_parameter_value().string_value
        
        self.frame_id = frame_id
        
        if msg_type == 'imu':
            self.pub = self.create_publisher(Imu, output_topic, 10)
            self.sub = self.create_subscription(Imu, input_topic, self.imu_callback, 10)
        else:
            self.pub = self.create_publisher(PointCloud2, output_topic, 10)
            self.sub = self.create_subscription(PointCloud2, input_topic, self.pointcloud_callback, 10)
        
        self.get_logger().info(f'Relaying {input_topic} -> {output_topic} with frame_id={frame_id}')

    def pointcloud_callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

    def imu_callback(self, msg):
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
