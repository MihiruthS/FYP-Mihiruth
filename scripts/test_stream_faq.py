"""Test FAQ with think_stream method (used by pipeline)."""
import sys
import asyncio
from pathlib import Path

project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / "src"))

from dotenv import load_dotenv
load_dotenv()

from robot_pipeline.pipeline import RobotVoicePipeline

async def test_stream():
    """Test FAQ with streaming responses."""
    print("="*60)
    print("Testing FAQ with think_stream() Method")
    print("="*60)
    
    # Initialize pipeline
    pipeline = RobotVoicePipeline()
    
    print("\n" + "="*60)
    print("Testing FAQ Questions with Streaming")
    print("="*60)
    
    # Test questions
    questions = [
        "Who is the head of the department?",
        "What is this department?",
        "Do you have postgraduate programs?",
        "Where is the computer lab?",
    ]
    
    for q in questions:
        print(f"\n❓ Question: {q}")
        response = ""
        async for chunk in pipeline.agent.think_stream(q):
            response += chunk
        print(f"✅ Response: {response[:100]}...")
    
    print("\n" + "="*60)
    print("Test Complete")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(test_stream())
