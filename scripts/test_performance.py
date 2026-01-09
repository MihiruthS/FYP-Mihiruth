"""Test performance improvements for non-FAQ questions."""
import sys
import asyncio
import time
from pathlib import Path

project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / "src"))

from dotenv import load_dotenv
load_dotenv()

from robot_pipeline.pipeline import RobotVoicePipeline

async def test_performance():
    """Test response times for non-FAQ questions."""
    print("="*60)
    print("Testing Performance Optimizations")
    print("="*60)
    
    # Initialize pipeline
    pipeline = RobotVoicePipeline()
    
    print("\n" + "="*60)
    print("Testing Non-FAQ Questions (RAG + LLM)")
    print("="*60)
    
    # Test non-FAQ questions that should use RAG
    questions = [
        "Tell me about the research facilities",
        "What industry sponsored labs do you have?",
        "Tell me about the electronic club",
        "What industry sponsored labs do you have?",  # Repeat to test cache
    ]
    
    for i, q in enumerate(questions, 1):
        print(f"\n{i}. Question: {q}")
        start_time = time.time()
        
        response = ""
        async for chunk in pipeline.agent.think_stream(q):
            response += chunk
        
        elapsed = time.time() - start_time
        print(f"   ⏱️  Time: {elapsed:.2f}s")
        print(f"   Response: {response[:80]}...")
    
    print("\n" + "="*60)
    print("Cache Stats")
    print("="*60)
    print(f"   Cached queries: {len(pipeline.agent._rag_cache)}")
    
    print("\n" + "="*60)
    print("Test Complete")
    print("="*60)

if __name__ == "__main__":
    asyncio.run(test_performance())
