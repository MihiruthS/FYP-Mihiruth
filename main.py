"""
Robot Voice Pipeline - Main Entry Point

Run the complete voice interaction pipeline for the robot.

Usage:
    python main.py
"""

import sys
from pathlib import Path

# Add src to path for imports
src_path = Path(__file__).parent / "src"
sys.path.insert(0, str(src_path))

from robot_pipeline.pipeline import RobotVoicePipeline


if __name__ == "__main__":
    import asyncio
    from dotenv import load_dotenv
    
    # Load environment variables
    load_dotenv()
    
    # Run the pipeline
    try:
        pipeline = RobotVoicePipeline()
        asyncio.run(pipeline.run())
    except KeyboardInterrupt:
        print("\nPipeline stopped by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
