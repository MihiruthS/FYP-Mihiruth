"""
ROS2 Entry Point for Robot Voice Pipeline

Runs the voice pipeline with ROS2 integration enabled.
Publishes transcripts, responses, and wake status to ROS2 topics.

Usage:
    python src/robot_pipeline/ros2_main.py
"""

import sys
from pathlib import Path
import asyncio
import rclpy
from rclpy.executors import MultiThreadedExecutor
from dotenv import load_dotenv
import threading

# Add src to path for imports
src_path = Path(__file__).parent.parent.parent / "src"
sys.path.insert(0, str(src_path))

from robot_pipeline.pipeline import RobotVoicePipeline
from robot_pipeline.ros2_bridge import VoicePipelineNode
from robot_pipeline.mouth_controller import MouthController


async def run_pipeline_with_ros2():
    """Run voice pipeline with ROS2 integration."""
    print("🤖 Starting Voice Pipeline with ROS2 Integration\n")
    
    # Initialize ROS2
    rclpy.init()
    
    # Create pipeline
    pipeline = RobotVoicePipeline()
    
    # Create ROS2 node and link to pipeline
    ros_node = VoicePipelineNode(pipeline)
    pipeline.ros_node = ros_node
    
    # Create mouth controller with ROS2 node
    # delay_ms: Adjust this value (50-200ms) to sync mouth with audio
    # Higher value = mouth moves later (use if mouth is too early)
    # Lower value = mouth moves earlier (use if mouth is too late)
    # mouth_opening_scale: Adjust mouth opening amount (1.0=normal, 1.5=50% more, 2.0=double)
    mouth_controller = MouthController(ros_node=ros_node, delay_ms=150, mouth_opening_scale=1.5)
    
    # Link mouth controller to audio playback
    pipeline.audio_playback.mouth_controller = mouth_controller
    
    # Link mouth controller to pipeline for emotion display
    pipeline.mouth_controller = mouth_controller
    
    # Start automatic blinking
    ros_node.start_blinking(mouth_controller)
    
    # Initialize motor speeds to 0 for smooth movement
    ros_node.publish_motor_speed(0, 0)  # Left jaw
    ros_node.publish_motor_speed(1, 0)  # Right jaw
    ros_node.publish_motor_speed(14, 0)  # Upper eyelid
    ros_node.publish_motor_speed(15, 0)  # Lower eyelid
    
    print("✅ Mouth Controller Initialized\n")
    
    # Store event loop for callbacks
    pipeline._loop = asyncio.get_event_loop()
    
    # Run ROS2 spin in separate thread
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    print("✅ ROS2 Node Running\n")
    
    try:
        # Run the main pipeline
        await pipeline.run()
    except KeyboardInterrupt:
        print("\n🛑 Pipeline stopped by user")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        ros_node.destroy_node()
        rclpy.shutdown()
        print("\n✅ ROS2 Node Shutdown")


if __name__ == '__main__':
    load_dotenv()
    
    try:
        asyncio.run(run_pipeline_with_ros2())
    except KeyboardInterrupt:
        print("\nExiting...")
