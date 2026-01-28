"""
Mouth Controller for Robot Voice Pipeline

Analyzes audio amplitude and controls jaw servo positions for lip sync.
"""
from rclpy.node import Node
import numpy as np
import asyncio
from collections import deque
from typing import Optional
import time
import random


class MouthController:
    """
    Controls robot mouth movement based on audio amplitude.
    
    Analyzes audio data and maps amplitude to jaw servo positions.
    Uses a delay buffer to sync with audio playback latency.
    """
    
    def __init__(self, ros_node=None, delay_ms: int = 100, mouth_opening_scale: float = 1.2):
        """
        Initialize mouth controller.
        
        Args:
            ros_node: ROS2 node with motor publishers (optional)
            delay_ms: Delay in milliseconds to sync with audio latency (default: 100ms)
            mouth_opening_scale: Multiplier for mouth opening (1.0=normal, 1.5=50% more, 2.0=double)
        """
        self.ros_node = ros_node
        self.delay_ms = delay_ms  # Delay to compensate for audio buffer latency
        self.mouth_opening_scale = mouth_opening_scale  # Scale factor for mouth opening
        
        # Eye lid servo channels
        self.upper_lid_channel = 14
        self.lower_lid_channel = 15
        
        # Buffer to hold delayed mouth positions
        self.position_buffer = deque(maxlen=20)  # Store recent positions
        self.buffer_delay_chunks = 0  # Will be calculated based on chunk timing
        
        # Jaw servo channels
        self.left_jaw_channel = 0
        self.right_jaw_channel = 1
        
        # Jaw position ranges
        self.left_jaw_min = 1296 * 4
        self.left_jaw_max = 1584 * 4
        self.right_jaw_min = 1712 * 4
        self.right_jaw_max = 1424 * 4
        
        # Amplitude threshold for mouth movement
        self.amplitude_max = 4000
        
        # Smoothing factor for mouth movement (0-1, higher = smoother)
        self.smoothing = 0.3
        self.last_jaw_position = 0
    
    def blink(self):
        """Perform a single blink action."""
        if not self.ros_node:
            return
        
        # Close eyelids
        self.ros_node.publish_motor_pos(self.upper_lid_channel, 2032*4)
        self.ros_node.publish_motor_pos(self.lower_lid_channel, 1584*4)
        
    def open_eyes(self):
        """Open eyelids to normal position."""
        if not self.ros_node:
            return
        
        # Open eyelids
        self.ros_node.publish_motor_pos(self.upper_lid_channel, 1200*4)
        self.ros_node.publish_motor_pos(self.lower_lid_channel, 1120*4)
    
        
    async def process_audio_chunk_delayed(self, audio_data: bytes):
        """
        Process audio chunk with delay compensation for better sync.
        
        Args:
            audio_data: PCM audio bytes (16-bit)
        """
        if not audio_data or not self.ros_node:
            return
        
        # Convert bytes to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        # Calculate amplitude (mean absolute value)
        amplitude = np.abs(audio_array).mean()
        
        # Clamp amplitude to max
        if amplitude > self.amplitude_max:
            amplitude = self.amplitude_max
        
        # Map amplitude to jaw position (0-100)
        target_jaw_position = int(np.interp(amplitude, [0, self.amplitude_max], [0, 100]))
        
        # Apply mouth opening scale to make mouth open more/less
        target_jaw_position = int(min(100, target_jaw_position * self.mouth_opening_scale))
        
        # Apply smoothing to reduce jitter
        smoothed_position = int(
            self.smoothing * self.last_jaw_position + 
            (1 - self.smoothing) * target_jaw_position
        )
        self.last_jaw_position = smoothed_position
        
        # Add to buffer with delay
        self.position_buffer.append(smoothed_position)
        
        # Apply delay compensation
        await asyncio.sleep(self.delay_ms / 1000.0)
        
        # Move jaw to the delayed position
        if len(self.position_buffer) > 0:
            delayed_position = self.position_buffer.popleft() if len(self.position_buffer) > 1 else smoothed_position
            self.move_jaw(delayed_position)
        
    def process_audio_chunk(self, audio_data: bytes):
        """
        Process audio chunk and move jaw based on amplitude.
        
        Args:
            audio_data: PCM audio bytes (16-bit)
        """
        if not audio_data or not self.ros_node:
            return
        
        # Convert bytes to numpy array
        audio_array = np.frombuffer(audio_data, dtype=np.int16)
        
        # Calculate amplitude (mean absolute value)
        amplitude = np.abs(audio_array).mean()
        
        # Clamp amplitude to max
        if amplitude > self.amplitude_max:
            amplitude = self.amplitude_max
        
        # Map amplitude to jaw position (0-100)
        jaw_position = int(np.interp(amplitude, [0, self.amplitude_max], [0, 100]))
        
        # Move jaw servos
        self.move_jaw(jaw_position)
    
    def move_jaw(self, jaw_position: int):
        """
        Move jaw servos to position based on amplitude.
        
        Args:
            jaw_position: Jaw opening (0=closed, 100=fully open)
        """
        if not self.ros_node:
            return
        
        # Map jaw position to servo positions
        left_jaw_pos = int(self._map_range(
            jaw_position, 0, 100, 
            self.left_jaw_min, self.left_jaw_max
        ))
        right_jaw_pos = int(self._map_range(
            jaw_position, 0, 100,
            self.right_jaw_min, self.right_jaw_max
        ))
        
        # Publish motor positions
        self.ros_node.publish_motor_pos(self.left_jaw_channel, left_jaw_pos)
        self.ros_node.publish_motor_pos(self.right_jaw_channel, right_jaw_pos)
    
    def close_mouth(self):
        """Close the mouth (jaw to rest position)."""
        if self.ros_node:
            self.ros_node.publish_motor_pos(self.left_jaw_channel, self.left_jaw_min)
            self.ros_node.publish_motor_pos(self.right_jaw_channel, self.right_jaw_min)
    
    def _map_range(self, value, in_min, in_max, out_min, out_max):
        """Map value from one range to another."""
        return out_min + (float(value - in_min) / float(in_max - in_min) * (out_max - out_min))
