"""
Initialize RAG Database for Robot Voice Pipeline

This script loads and indexes all documents from the vector_database_documents
folder into ChromaDB for Retrieval-Augmented Generation (RAG).

Usage:
    python init_rag_database.py [--reset]
    
Options:
    --reset    Clear existing database and rebuild from scratch
"""

import os
import sys
import argparse
from pathlib import Path
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add src directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from robot_pipeline.ai.rag_database import RAGDatabase


def main():
    """Initialize or reset the RAG database."""
    parser = argparse.ArgumentParser(description="Initialize RAG Database")
    parser.add_argument(
        '--reset',
        action='store_true',
        help='Clear existing database and rebuild from scratch'
    )
    parser.add_argument(
        '--docs-dir',
        type=str,
        default='./src/knowledge_base/documents',
        help='Path to documents directory (default: ./src/knowledge_base/documents)'
    )
    parser.add_argument(
        '--db-dir',
        type=str,
        default='./data/chroma_db',
        help='Path to ChromaDB directory (default: ./data/chroma_db)'
    )
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("🚀 RAG Database Initialization")
    print("=" * 60)
    
    # Check if OpenAI API key is set
    if not os.getenv("OPENAI_API_KEY"):
        print("❌ Error: OPENAI_API_KEY not found in environment variables")
        print("   Please add it to your .env file or export it:")
        print("   export OPENAI_API_KEY='your-api-key-here'")
        sys.exit(1)
    
    # Check if documents directory exists
    docs_path = Path(args.docs_dir)
    if not docs_path.exists():
        print(f"❌ Error: Documents directory not found: {args.docs_dir}")
        sys.exit(1)
    
    # Count documents
    txt_files = list(docs_path.glob("*.txt"))
    print(f"📚 Found {len(txt_files)} document(s) in {args.docs_dir}")
    if not txt_files:
        print("⚠️  Warning: No .txt files found in documents directory")
        sys.exit(1)
    
    # Initialize RAG database
    print("\n🔧 Initializing RAG Database...")
    
    try:
        rag_db = RAGDatabase(
            persist_directory=args.db_dir,
            documents_directory=args.docs_dir,
            chunk_size=1000,
            chunk_overlap=200
        )
        
        # Reset if requested
        if args.reset:
            print("\n🗑️  Resetting database...")
            rag_db.clear_database()
            print("✅ Database cleared")
            
            print("\n📦 Loading and indexing documents...")
            rag_db.load_documents()
        
        # Show statistics
        print("\n" + "=" * 60)
        print("📊 Database Statistics")
        print("=" * 60)
        
        stats = rag_db.get_stats()
        print(f"Status: {stats.get('status', 'unknown')}")
        print(f"Total chunks: {stats.get('total_chunks', 0)}")
        print(f"Storage: {stats.get('persist_directory', 'N/A')}")
        
        # Test retrieval
        print("\n" + "=" * 60)
        print("🧪 Testing Retrieval")
        print("=" * 60)
        
        test_queries = [
            "What is the department about?",
            "Who is the head of department?",
            "What programs are offered?"
        ]
        
        for query in test_queries:
            print(f"\nQuery: {query}")
            docs = rag_db.retrieve(query, k=2)
            if docs:
                print(f"  ✓ Retrieved {len(docs)} relevant chunks")
                # Show a snippet of first result
                snippet = docs[0].page_content[:150].replace('\n', ' ')
                print(f"  Preview: {snippet}...")
            else:
                print("  ✗ No documents retrieved")
        
        print("\n" + "=" * 60)
        print("✅ RAG Database initialized successfully!")
        print("=" * 60)
        print("\nYou can now run the robot voice pipeline with RAG enabled:")
        print("  python main.py")
        
    except Exception as e:
        print(f"\n❌ Error initializing database: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()
