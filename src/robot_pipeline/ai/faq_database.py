"""
FAQ Database for Robot Voice Pipeline

Provides fast responses for frequently asked questions using
embedding-based similarity matching. Falls back to RAG if no match found.
"""

import os
import json
from pathlib import Path
from typing import Optional, List, Dict, Tuple
from dataclasses import dataclass

from langchain_openai import OpenAIEmbeddings
import numpy as np


@dataclass
class FAQItem:
    """Single FAQ entry with question, answer, and embedding."""
    question: str
    answer: str
    embedding: Optional[np.ndarray] = None
    category: str = "general"
    keywords: List[str] = None


class FAQDatabase:
    """
    Fast FAQ lookup using semantic similarity.
    
    Matches user queries against pre-defined FAQs using embeddings.
    Returns cached answers instantly without calling LLM.
    """
    
    def __init__(
        self,
        api_key: Optional[str] = None,
        similarity_threshold: float = 0.82,  # High threshold for accurate matches
        faq_file: Optional[str] = None,
    ):
        """
        Initialize FAQ database.
        
        Args:
            api_key: OpenAI API key for embeddings
            similarity_threshold: Minimum similarity (0-1) to return FAQ answer
            faq_file: Path to JSON file containing FAQ data (defaults to data/faqs.json)
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            # Don't raise error, just disable FAQ functionality
            print("OPENAI_API_KEY not found - FAQ database disabled")
            self.embeddings_model = None
            self.faqs = []
            return
        
        self.similarity_threshold = similarity_threshold
        
        # Set default FAQ file path
        if faq_file is None:
            # Get project root (4 levels up from this file)
            project_root = Path(__file__).parent.parent.parent.parent
            self.faq_file = project_root / "data" / "faqs.json"
        else:
            self.faq_file = Path(faq_file)
        
        self.embeddings_model = OpenAIEmbeddings(
            api_key=self.api_key,
            model="text-embedding-3-small"
        )
        
        # Load FAQs
        self.faqs: List[FAQItem] = []
        self._load_faqs()
        self._compute_embeddings()
        
        print(f"FAQ Database loaded with {len(self.faqs)} entries")
    
    def _load_faqs(self):
        """Load FAQs from JSON file."""
        if not self.faq_file.exists():
            print(f"FAQ file not found: {self.faq_file}")
            print("   Creating default FAQ file...")
            self._create_default_faq_file()
        
        # Check for cached embeddings file
        cache_file = self.faq_file.with_suffix('.embeddings.npy')
        embeddings_cache = None
        
        try:
            if cache_file.exists():
                # Load cached embeddings
                embeddings_cache = np.load(cache_file, allow_pickle=True).item()
                print(f"Loaded cached FAQ embeddings from {cache_file.name}")
        except Exception as e:
            print(f"Could not load embeddings cache: {e}")
            embeddings_cache = None
        
        try:
            with open(self.faq_file, 'r', encoding='utf-8') as f:
                faq_data = json.load(f)
            
            print(f"Loaded {len(faq_data)} FAQs from {self.faq_file.name}")
            
            # Convert to FAQItem objects
            for item in faq_data:
                question = item["question"]
                faq = FAQItem(
                    question=question,
                    answer=item["answer"],
                    category=item.get("category", "general"),
                    keywords=item.get("keywords", [])
                )
                
                # Use cached embedding if available
                if embeddings_cache and question in embeddings_cache:
                    faq.embedding = embeddings_cache[question]
                
                self.faqs.append(faq)
        except json.JSONDecodeError as e:
            print(f"Error parsing FAQ file: {e}")
        except Exception as e:
            print(f"Error loading FAQs: {e}")
    
    def _create_default_faq_file(self):
        """Create default FAQ file if it doesn't exist."""
        default_faqs = [
            {
                "question": "Who is the head of the department?",
                "answer": "The Head of the Department is Dr. Thayaparan Subramaniam.",
                "category": "staff",
                "keywords": ["head", "hod", "head of department"]
            },
            {
                "question": "What is this department?",
                "answer": "This is the Electronic and Telecommunication Engineering Department at the University of Moratuwa.",
                "category": "general",
                "keywords": ["department", "what is", "entc"]
            }
        ]
        
        # Create directory if it doesn't exist
        self.faq_file.parent.mkdir(parents=True, exist_ok=True)
        
        # Write default FAQs
        with open(self.faq_file, 'w', encoding='utf-8') as f:
            json.dump(default_faqs, f, indent=4, ensure_ascii=False)
        
        print(f"Created default FAQ file: {self.faq_file}")
    
    def _compute_embeddings(self):
        """Compute embeddings for all FAQ questions."""
        # Check if any FAQs need embeddings computed
        needs_computation = [faq for faq in self.faqs if faq.embedding is None]
        
        if not needs_computation:
            print(f"All {len(self.faqs)} FAQ embeddings loaded from cache")
            return
        
        print(f"Computing embeddings for {len(needs_computation)} new FAQs...")
        
        questions_to_compute = [faq.question for faq in needs_computation]
        embeddings = self.embeddings_model.embed_documents(questions_to_compute)
        
        for i, faq in enumerate(needs_computation):
            faq.embedding = np.array(embeddings[i])
        
        # Save all embeddings to cache for next time
        self._save_embeddings_cache()
        
        print(f"Computed and cached {len(embeddings)} FAQ embeddings")
    
    def _save_embeddings_cache(self):
        """Save computed embeddings to cache file."""
        try:
            cache_file = self.faq_file.with_suffix('.embeddings.npy')
            embeddings_dict = {
                faq.question: faq.embedding 
                for faq in self.faqs 
                if faq.embedding is not None
            }
            np.save(cache_file, embeddings_dict, allow_pickle=True)
            print(f"Saved {len(embeddings_dict)} embeddings to cache")
        except Exception as e:
            print(f"Could not save embeddings cache: {e}")
    
    def _cosine_similarity(self, vec1: np.ndarray, vec2: np.ndarray) -> float:
        """Calculate cosine similarity between two vectors."""
        dot_product = np.dot(vec1, vec2)
        norm1 = np.linalg.norm(vec1)
        norm2 = np.linalg.norm(vec2)
        return dot_product / (norm1 * norm2) if norm1 and norm2 else 0.0
    
    def search(self, query: str) -> Optional[Tuple[str, float, str]]:
        """
        Search for matching FAQ.
        
        Args:
            query: User's question
        
        Returns:
            Tuple of (answer, similarity_score, matched_question) if found, else None
        """
        if not query.strip() or not self.embeddings_model or not self.faqs:
            return None
        
        # Get query embedding
        query_embedding = np.array(self.embeddings_model.embed_query(query))
        
        # Find best match
        best_match = None
        best_score = 0.0
        best_question = ""
        
        for faq in self.faqs:
            if faq.embedding is None:
                continue
            
            similarity = self._cosine_similarity(query_embedding, faq.embedding)
            
            # Boost score if keywords match
            query_lower = query.lower()
            keyword_boost = 0.0
            if faq.keywords:
                for keyword in faq.keywords:
                    if keyword.lower() in query_lower:
                        keyword_boost = 0.05
                        break
            
            total_score = similarity + keyword_boost
            
            if total_score > best_score:
                best_score = total_score
                best_match = faq.answer
                best_question = faq.question
        
        # Return if above threshold
        if best_score >= self.similarity_threshold:
            print(f"FAQ Match! Score: {best_score:.3f}, Question: '{best_question}'")
            return (best_match, best_score, best_question)
        
        return None
    
    def get_answer(self, query: str) -> Optional[str]:
        """
        Get FAQ answer if query matches.
        
        Args:
            query: User's question
        
        Returns:
            Answer string if match found, else None
        """
        result = self.search(query)
        return result[0] if result else None
    
    def add_faq(self, question: str, answer: str, category: str = "general"):
        """
        Add a new FAQ dynamically.
        
        Args:
            question: FAQ question
            answer: FAQ answer
            category: FAQ category
        """
        # Compute embedding
        embedding = np.array(self.embeddings_model.embed_query(question))
        
        # Add to database
        faq = FAQItem(
            question=question,
            answer=answer,
            category=category,
            embedding=embedding
        )
        self.faqs.append(faq)
        print(f"Added FAQ: '{question}'")
    
    def get_stats(self) -> Dict[str, any]:
        """Get FAQ database statistics."""
        categories = {}
        for faq in self.faqs:
            categories[faq.category] = categories.get(faq.category, 0) + 1
        
        return {
            "total_faqs": len(self.faqs),
            "categories": categories,
            "threshold": self.similarity_threshold
        }
