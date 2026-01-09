"""Test FAQ with the exact queries from the user's log."""
import sys
from pathlib import Path

project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / "src"))

from dotenv import load_dotenv
load_dotenv()

from robot_pipeline.ai.faq_database import FAQDatabase

print("="*60)
print("Testing Exact User Queries from Log")
print("="*60)

# Initialize FAQ
faq = FAQDatabase()

# Test queries - EXACT from user's log
queries = [
    "What is this department?",  # From: 📝 You said: 'What is this department?'
    "Do you have postgraduate programs?",  # From: 📝 You said: 'Do you have postgraduate programs?'
    "Where is the computer lab?",  # From: 📝 You said: 'Where is the computer lab?'
]

for query in queries:
    print(f"\n{'='*60}")
    print(f"Query: '{query}'")
    print(f"{'='*60}")
    
    result = faq.search(query)
    
    if result:
        answer, score, matched_q = result
        print(f"✅ MATCH FOUND")
        print(f"   Score: {score:.3f}")
        print(f"   Matched: '{matched_q}'")
        print(f"   Answer: {answer[:80]}...")
    else:
        print(f"❌ NO MATCH (threshold: {faq.similarity_threshold})")
        
        # Show closest match even if below threshold
        print("\n🔍 Finding closest match...")
        query_embedding = faq.embeddings_model.embed_query(query)
        import numpy as np
        query_emb = np.array(query_embedding)
        
        best_score = 0
        best_q = ""
        for faq_item in faq.faqs:
            similarity = faq._cosine_similarity(query_emb, faq_item.embedding)
            if similarity > best_score:
                best_score = similarity
                best_q = faq_item.question
        
        print(f"   Closest: '{best_q}' (score: {best_score:.3f})")

print("\n" + "="*60)
print("Test Complete")
print("="*60)
