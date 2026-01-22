"""
Debug script to test RAG retrieval and see what context is being retrieved.
"""

import sys
from pathlib import Path

# Add src to path
src_path = Path(__file__).parent.parent / "src"
sys.path.insert(0, str(src_path))

from robot_pipeline.ai.rag_database import RAGDatabase
from dotenv import load_dotenv

load_dotenv()

PROJECT_ROOT = Path(__file__).parent.parent

def test_rag_retrieval():
    """Test what RAG retrieves for specific queries."""
    
    print("Initializing RAG Database...")
    rag = RAGDatabase(
        persist_directory=str(PROJECT_ROOT / "data" / "chroma_db"),
        documents_directory=str(PROJECT_ROOT / "src" / "knowledge_base" / "documents")
    )
    
    test_queries = [
        "analog electronic lab",
        "Who is the president of the Electronic Club",
        "What labs are available?",
        "Where is the analog lab located?"
    ]
    
    print("\n" + "="*80)
    for query in test_queries:
        print(f"\nQuery: '{query}'")
        print("-"*80)
        
        # Get raw documents first
        docs = rag.retrieve(query, k=2)
        print(f"Retrieved {len(docs)} documents")
        
        for i, doc in enumerate(docs, 1):
            print(f"\n  Document {i}:")
            print(f"  Source: {doc.metadata.get('source', 'Unknown')}")
            print(f"  Content length: {len(doc.page_content)} chars")
            print(f"  Content preview: {doc.page_content[:200]}...")
        
        # Get context like the agent does
        context = rag.get_context_string(query, k=2, max_length=800)
        
        print(f"\nFinal Context String ({len(context)} chars):")
        if context:
            print(context[:600] + "..." if len(context) > 600 else context)
        else:
            print("EMPTY!")
        
        print("="*80)

if __name__ == "__main__":
    test_rag_retrieval()
