"""
ROS2 Bridge for Robot Voice Pipeline

Publishes escort destination locations and motor positions to ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
import random
import time


class VoicePipelineNode(Node):
    """
    ROS2 Node wrapper for the Voice Pipeline.
    
    Publishers:
        /robot/locations (String): Escort destination location
        /motor_pos (Int32MultiArray): Motor positions [channel, position]
        /motor_speed (Int32MultiArray): Motor speeds [channel, speed]
    """
    
    def __init__(self, pipeline):
        super().__init__('voice_pipeline_node')
        self.pipeline = pipeline
        self.mouth_controller = None  # Will be set later
        
        # Publishers
        self.location_pub = self.create_publisher(
            String, '/robot/locations', 10
        )
        self.motor_pos_pub = self.create_publisher(
            Int32MultiArray, '/motor_pos', 10
        )
        self.motor_speed_pub = self.create_publisher(
            Int32MultiArray, '/motor_speed', 10
        )
        
        # Blinking timer - will be started after mouth_controller is set
        self.blink_timer = None
        
        self.get_logger().info('Voice Pipeline ROS2 Node initialized')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /robot/locations')
        self.get_logger().info('  - /motor_pos')
        self.get_logger().info('  - /motor_speed')
    
    def publish_location(self, location: str):
        """Publish escort destination location."""
        msg = String()
        msg.data = location
        self.location_pub.publish(msg)
        self.get_logger().info(f'📤 Location: {location}')
    
    def publish_motor_pos(self, channel: int, position: int):
        """Publish motor position."""
        msg = Int32MultiArray()
        msg.data = [channel, position]
        self.motor_pos_pub.publish(msg)
    
    def publish_motor_speed(self, channel: int, speed: int):
        """Publish motor speed."""
        msg = Int32MultiArray()
        msg.data = [channel, speed]
        self.motor_speed_pub.publish(msg)
    
    def start_blinking(self, mouth_controller):
        """Start automatic blinking timer."""
        self.mouth_controller = mouth_controller
        interval = random.uniform(2.0, 4.0)  # Random interval between 2 to 4 seconds
        self.blink_timer = self.create_timer(interval, self.blink_callback)
        self.get_logger().info('Eye blinking started')
    
    def blink_callback(self):
        """Perform blinking action."""
        if not self.mouth_controller:
            return
        
        # Blink
        self.mouth_controller.blink()
        time.sleep(0.2)
        self.mouth_controller.open_eyes()
        
        # Cancel old timer and create new one with random interval
        if self.blink_timer:
            self.blink_timer.cancel()
        interval = random.uniform(2.0, 4.0)
        self.blink_timer = self.create_timer(interval, self.blink_callback)
