"""
Test FAQ Database

Tests the FAQ system with common questions to verify fast responses.
"""

import sys
from pathlib import Path
import asyncio
from dotenv import load_dotenv

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

load_dotenv()

from robot_pipeline.ai.faq_database import FAQDatabase
from robot_pipeline.ai.agent import AIAgent
from robot_pipeline.ai.prompts import PromptGenerator


def test_faq_database():
    """Test FAQ database directly."""
    print("=" * 60)
    print("🧪 Testing FAQ Database")
    print("=" * 60)
    
    # Initialize FAQ database
    print("\n1️⃣ Initializing FAQ Database...")
    faq = FAQDatabase()
    
    stats = faq.get_stats()
    print(f"   ✓ Loaded {stats['total_faqs']} FAQs")
    print(f"   ✓ Categories: {list(stats['categories'].keys())}")
    print(f"   ✓ Threshold: {stats['threshold']}")
    
    # Test queries
    test_queries = [
        "Who is the head of the department?",
        "who is the head of department",  # Case variation
        "Who's the HOD?",  # Abbreviation
        "tell me about the head",  # Different wording
        "What programs do you offer?",
        "what courses are available",  # Different wording
        "Tell me about the Electronic Club",
        "what is the electronic club",  # Case variation
        "Where is the computer lab?",
        "can you direct me to the computer lab",  # Different phrasing
        "What is your name?",
        "who are you",
        "What is ENTC?",
        "This is a completely random question that won't match",  # Should not match
    ]
    
    print("\n2️⃣ Testing FAQ Matches...")
    print("=" * 60)
    
    matches = 0
    no_matches = 0
    
    for query in test_queries:
        print(f"\n📝 Query: \"{query}\"")
        result = faq.search(query)
        
        if result:
            answer, score, matched_question = result
            matches += 1
            print(f"   ✅ MATCH (score: {score:.3f})")
            print(f"   Matched: \"{matched_question}\"")
            print(f"   Answer: \"{answer}\"")
        else:
            no_matches += 1
            print(f"   ❌ NO MATCH (below threshold)")
    
    print("\n" + "=" * 60)
    print(f"📊 Results: {matches} matches, {no_matches} no matches")
    print("=" * 60)


async def test_faq_integration():
    """Test FAQ integration with AI agent."""
    print("\n\n" + "=" * 60)
    print("🧪 Testing FAQ + AI Agent Integration")
    print("=" * 60)
    
    # Initialize components
    print("\n1️⃣ Initializing components...")
    faq_db = FAQDatabase()
    prompt_gen = PromptGenerator()
    
    agent = AIAgent(
        prompt_generator=prompt_gen,
        faq_database=faq_db,
        use_faq=True
    )
    print("   ✓ AI Agent initialized with FAQ")
    
    # Test queries
    test_queries = [
        "Who is the head of the department?",  # Should hit FAQ
        "What programs do you offer?",  # Should hit FAQ
        "What is the meaning of life?",  # Should fall back to LLM
    ]
    
    print("\n2️⃣ Testing responses...")
    print("=" * 60)
    
    for i, query in enumerate(test_queries, 1):
        print(f"\n[Test {i}] Query: \"{query}\"")
        print("-" * 60)
        
        import time
        start = time.time()
        response = await agent.think(query)
        elapsed = time.time() - start
        
        print(f"🤖 Response: {response}")
        print(f"⏱️  Time: {elapsed:.3f}s")
        print("=" * 60)
    
    print("\n✅ FAQ Integration Test Complete!")


if __name__ == "__main__":
    # Test FAQ database
    test_faq_database()
    
    # Test integration with AI agent
    asyncio.run(test_faq_integration())
