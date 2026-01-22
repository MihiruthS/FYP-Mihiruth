"""Test FAQ through main pipeline without audio."""
import sys
import asyncio
from pathlib import Path

# Same setup as main.py
src_path = Path(__file__).parent.parent / "src"
sys.path.insert(0, str(src_path))

from dotenv import load_dotenv
load_dotenv()

from robot_pipeline.pipeline import RobotVoicePipeline

async def test_faq():
    """Test FAQ responses through the pipeline."""
    print("="*60)
    print("Testing FAQ through Main Pipeline")
    print("="*60)
    
    # Initialize pipeline (same as main.py)
    pipeline = RobotVoicePipeline()
    
    print("\n" + "="*60)
    print("Testing FAQ Questions")
    print("="*60)
    
    # Test questions
    questions = [
        "Who is the head of the department?",
        "What is ENTC?",
        "Where is the computer lab?",
        "What programs do you offer?",
    ]
    
    for q in questions:
        print(f"\nQuestion: {q}")
        response = await pipeline.agent.think(q)
        print(f"Response: {response[:100]}...")
    
    print("\n" + "="*60)
    print("Test Complete")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(test_faq())
