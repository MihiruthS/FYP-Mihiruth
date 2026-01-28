"""
ROS2 Bridge for Robot Voice Pipeline

Publishes escort destination locations and motor positions to ROS2 topics.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Bool
import random
import time


class VoicePipelineNode(Node):
    """
    ROS2 Node wrapper for the Voice Pipeline.
    
    Publishers:
        /locations (String): Escort destination location
        /motor_pos (Int32MultiArray): Motor positions [channel, position]
        /motor_speed (Int32MultiArray): Motor speeds [channel, speed]
    
    Subscribers:
        /arrived (Bool): Notification when robot arrives at destination
    """
    
    def __init__(self, pipeline):
        super().__init__('voice_pipeline_node')
        self.pipeline = pipeline
        self.mouth_controller = None  # Will be set later
        
        # Track escorting state
        self.is_escorting = False
        self.current_destination = None
        
        # Publishers
        self.location_pub = self.create_publisher(
            String, '/locations', 10
        )
        self.motor_pos_pub = self.create_publisher(
            Int32MultiArray, '/motor_pos', 10
        )
        self.motor_speed_pub = self.create_publisher(
            Int32MultiArray, '/motor_speed', 10
        )
        
        # Subscribers
        self.arrived_sub = self.create_subscription(
            Bool,
            '/arrived',
            self.arrived_callback,
            10
        )
        
        # Blinking timer - will be started after mouth_controller is set
        self.blink_timer = None
        
        self.get_logger().info('Voice Pipeline ROS2 Node initialized')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - /locations')
        self.get_logger().info('  - /motor_pos')
        self.get_logger().info('  - /motor_speed')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /arrived')
    
    def publish_location(self, location: str):
        """Publish escort destination location and pause conversation."""
        msg = String()
        msg.data = location
        self.location_pub.publish(msg)
        self.is_escorting = True
        self.current_destination = location
        self.pipeline.is_escorting = True  # Pause conversational pipeline
        self.get_logger().info(f'📤 Location: {location}')
        self.get_logger().info(f'🚶 Escorting started - conversational pipeline paused')
    
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
    
    def arrived_callback(self, msg: Bool):
        """Handle arrival notification from /arrived topic."""
        if msg.data and self.is_escorting:
            self.get_logger().info(f'✅ Arrived at destination: {self.current_destination}')
            
            # Format location name for speech
            location_name = self.current_destination.replace('_', ' ')
            arrival_message = f"You have arrived at the {location_name}"
            
            # Announce arrival and resume pipeline
            if hasattr(self.pipeline, '_loop') and self.pipeline._loop:
                # Schedule the speak task in the pipeline's event loop
                asyncio = __import__('asyncio')
                future = asyncio.run_coroutine_threadsafe(
                    self.pipeline._speak(arrival_message),
                    self.pipeline._loop
                )
                # Wait for speech to complete, then resume
                def resume_pipeline(fut):
                    self.is_escorting = False
                    self.pipeline.is_escorting = False
                    self.current_destination = None
                    self.get_logger().info('🎤 Conversational pipeline resumed')
                
                future.add_done_callback(resume_pipeline)
            else:
                # Fallback: just resume pipeline
                self.is_escorting = False
                self.pipeline.is_escorting = False
                self.current_destination = None
                self.get_logger().info('⚠️ Could not announce arrival - pipeline loop not available')
    
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
