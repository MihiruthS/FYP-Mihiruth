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
