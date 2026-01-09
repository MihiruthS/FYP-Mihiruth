"""
Test RAG Integration

Quick test script to verify RAG database integration with the AI agent.
"""

import os
import asyncio
from dotenv import load_dotenv
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / 'src'))

load_dotenv()

from robot_pipeline.ai.rag_database import RAGDatabase
from robot_pipeline.ai.agent import AIAgent
from robot_pipeline.ai.prompts import PromptGenerator


async def test_rag_integration():
    """Test RAG integration with AI Agent."""
    
    print("=" * 60)
    print("🧪 Testing RAG Integration")
    print("=" * 60)
    
    # Initialize components
    print("\n1️⃣ Initializing RAG Database...")
    rag_db = RAGDatabase(
        persist_directory="./data/chroma_db",
        documents_directory="./src/knowledge_base/documents"
    )
    
    stats = rag_db.get_stats()
    print(f"   ✓ Database ready with {stats.get('total_chunks', 0)} chunks")
    
    print("\n2️⃣ Initializing AI Agent with RAG...")
    prompt_gen = PromptGenerator()
    agent = AIAgent(
        prompt_generator=prompt_gen,
        rag_database=rag_db,
        use_rag=True
    )
    print("   ✓ AI Agent initialized")
    
    # Test queries
    test_queries = [
        "Who is the head of the department?",
        "What programs does the department offer?",
        "Tell me about the Electronic Club",
        "What research facilities are available?",
    ]
    
    print("\n3️⃣ Testing RAG-enhanced responses...")
    print("=" * 60)
    
    for i, query in enumerate(test_queries, 1):
        print(f"\n[Query {i}] {query}")
        print("-" * 60)
        
        # Get RAG context
        context = rag_db.get_context_string(query, k=2, max_length=500)
        if context:
            print("📚 Retrieved Context:")
            # Show first 150 chars of context
            context_preview = context.split('\n')[1][:150] if '\n' in context else context[:150]
            print(f"   {context_preview}...")
        
        # Get AI response
        response = await agent.think(query)
        print(f"\n🤖 Response: {response}")
        print("=" * 60)
    
    print("\n✅ RAG Integration Test Complete!")
    print("\nThe system is ready to use. Run 'python main.py' to start the voice pipeline.")


if __name__ == "__main__":
    asyncio.run(test_rag_integration())
