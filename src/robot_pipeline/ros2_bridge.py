"""
ROS2 Bridge for Robot Voice Pipeline

Publishes escort destination locations to ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoicePipelineNode(Node):
    """
    ROS2 Node wrapper for the Voice Pipeline.
    
    Publishers:
        /robot/locations (String): Escort destination location
    """
    
    def __init__(self, pipeline):
        super().__init__('voice_pipeline_node')
        self.pipeline = pipeline
        
        # Publishers
        self.location_pub = self.create_publisher(
            String, '/robot/locations', 10
        )
        
        self.get_logger().info('Voice Pipeline ROS2 Node initialized')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /robot/locations')
    
    def publish_location(self, location: str):
        """Publish escort destination location."""
        msg = String()
        msg.data = location
        self.location_pub.publish(msg)
        self.get_logger().info(f'📤 Location: {location}')
