#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class TFRepublisher(Node):
    def __init__(self):
        super().__init__('tf_republisher')
        
        # Subscribe to Gazebo's TF (with my_robot/ prefix)
        self.subscription = self.create_subscription(
            TFMessage,
            '/gz_tf',
            self.tf_callback,
            10
        )
        
        # Publisher to clean TF topic (without prefix)
        self.publisher = self.create_publisher(
            TFMessage,
            '/tf',
            10
        )
        
        self.get_logger().info('TF Republisher started - stripping my_robot/ prefix from frames')
    
    def tf_callback(self, msg):
        # Create new TFMessage with cleaned frame names
        cleaned_msg = TFMessage()
        
        for transform in msg.transforms:
            new_transform = TransformStamped()
            
            # Copy timestamp
            new_transform.header.stamp = transform.header.stamp
            
            # Strip 'my_robot/' prefix from frame names
            new_transform.header.frame_id = transform.header.frame_id.replace('my_robot/', '')
            new_transform.child_frame_id = transform.child_frame_id.replace('my_robot/', '')
            
            # Copy transform data
            new_transform.transform = transform.transform
            
            cleaned_msg.transforms.append(new_transform)
        
        # Publish cleaned transforms
        if cleaned_msg.transforms:
            self.publisher.publish(cleaned_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TFRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
