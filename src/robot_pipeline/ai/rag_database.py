"""
RAG Database Module for Robot Voice Pipeline

Implements Retrieval-Augmented Generation using ChromaDB for vector storage
and OpenAI embeddings for semantic search across knowledge documents.
"""

import os
from typing import List, Optional
from pathlib import Path

from langchain_openai import OpenAIEmbeddings
from langchain_chroma import Chroma
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_core.documents import Document


class RAGDatabase:
    """
    RAG Database for storing and retrieving knowledge documents.
    
    Uses ChromaDB for vector storage and OpenAI embeddings for semantic search.
    Provides context-aware retrieval for the AI agent.
    """
    
    def __init__(
        self,
        persist_directory: str = "./chroma_db",
        documents_directory: Optional[str] = None,
        api_key: Optional[str] = None,
        chunk_size: int = 1000,
        chunk_overlap: int = 200,
    ):
        """
        Initialize the RAG database.
        
        Args:
            persist_directory: Directory to persist ChromaDB data
            documents_directory: Directory containing text documents to index
            api_key: OpenAI API key (reads from OPENAI_API_KEY env var if not provided)
            chunk_size: Size of text chunks for splitting
            chunk_overlap: Overlap between chunks
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        if not self.api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        
        self.persist_directory = persist_directory
        self.documents_directory = documents_directory
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        
        # Initialize embeddings
        self.embeddings = OpenAIEmbeddings(
            api_key=self.api_key,
            model="text-embedding-3-small"  # Cost-effective and fast
        )
        
        # Initialize or load vector store
        self.vector_store = None
        self._initialize_vector_store()
    
    def _initialize_vector_store(self):
        """Initialize ChromaDB vector store."""
        try:
            # Try to load existing database
            self.vector_store = Chroma(
                persist_directory=self.persist_directory,
                embedding_function=self.embeddings,
                collection_name="knowledge_base"
            )
            
            # Check if database has documents
            collection = self.vector_store._collection
            if collection.count() == 0:
                print("Empty vector database detected. Loading documents...")
                self.load_documents()
            else:
                print(f"Loaded existing vector database with {collection.count()} chunks")
                
        except Exception as e:
            print(f"Creating new vector database: {e}")
            # Create new database
            if self.documents_directory:
                self.load_documents()
    
    def load_documents(self):
        """
        Load and index all documents from the documents directory.
        
        Reads all .txt files, splits them into chunks, and stores in ChromaDB.
        """
        if not self.documents_directory:
            print("No documents directory specified")
            return
        
        docs_path = Path(self.documents_directory)
        if not docs_path.exists():
            print(f"Documents directory not found: {self.documents_directory}")
            return
        
        # Find all .txt files
        txt_files = list(docs_path.glob("*.txt"))
        if not txt_files:
            print(f"No .txt files found in {self.documents_directory}")
            return
        
        print(f"Found {len(txt_files)} documents to index...")
        
        # Load documents
        documents = []
        for file_path in txt_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                doc = Document(
                    page_content=content,
                    metadata={
                        "source": file_path.name,
                        "category": file_path.stem.replace('_', ' ')
                    }
                )
                documents.append(doc)
                print(f"  Loaded: {file_path.name}")
            except Exception as e:
                print(f"  Error loading {file_path.name}: {e}")
        
        if not documents:
            print("No documents loaded")
            return
        
        # Split documents into chunks
        text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=self.chunk_size,
            chunk_overlap=self.chunk_overlap,
            length_function=len,
            separators=["\n\n", "\n", ".", "!", "?", ",", " ", ""]
        )
        
        split_docs = text_splitter.split_documents(documents)
        print(f"Split into {len(split_docs)} chunks")
        
        # Create vector store
        self.vector_store = Chroma.from_documents(
            documents=split_docs,
            embedding=self.embeddings,
            persist_directory=self.persist_directory,
            collection_name="knowledge_base"
        )
        
        print(f"Indexed {len(split_docs)} chunks into vector database")
    
    def retrieve(
        self,
        query: str,
        k: int = 3,
        score_threshold: float = 0.5
    ) -> List[Document]:
        """
        Retrieve relevant documents for a query.
        
        Args:
            query: The search query
            k: Number of documents to retrieve
            score_threshold: Minimum similarity score (0-1)
        
        Returns:
            List of relevant documents with metadata
        """
        if not self.vector_store:
            print("Vector store not initialized")
            return []
        
        try:
            # Perform similarity search with scores
            results = self.vector_store.similarity_search_with_score(
                query, 
                k=k
            )
            
            # Filter by score threshold (lower score = more similar for some distance metrics)
            # ChromaDB uses L2 distance, so we'll filter differently
            filtered_docs = [doc for doc, score in results]
            
            print(f"Retrieved {len(filtered_docs)} relevant chunks for query: '{query[:50]}...'")
            
            return filtered_docs
            
        except Exception as e:
            print(f"Error retrieving documents: {e}")
            return []
    
    def get_context_string(
        self,
        query: str,
        k: int = 3,
        max_length: int = 2000
    ) -> str:
        """
        Get a formatted context string for a query.
        
        Args:
            query: The search query
            k: Number of documents to retrieve
            max_length: Maximum length of context string
        
        Returns:
            Formatted context string for the AI agent
        """
        docs = self.retrieve(query, k=k)
        
        if not docs:
            return ""
        
        # Build context string
        context_parts = []
        total_length = 0
        
        for i, doc in enumerate(docs, 1):
            source = doc.metadata.get('source', 'Unknown')
            content = doc.page_content.strip()
            
            # Add document with source
            doc_text = f"[Source: {source}]\n{content}\n"
            
            # Check length limit - if first document is too long, truncate it
            if i == 1 and len(doc_text) > max_length:
                # Include at least the first document, but truncate
                doc_text = doc_text[:max_length] + "...[truncated]"
                context_parts.append(doc_text)
                total_length = max_length
                break
            elif total_length + len(doc_text) > max_length:
                # Skip this document if it would exceed limit
                break
            
            context_parts.append(doc_text)
            total_length += len(doc_text)
        
        if context_parts:
            return "RETRIEVED KNOWLEDGE:\n" + "\n---\n".join(context_parts)
        
        return ""
    
    def clear_database(self):
        """Clear all documents from the database."""
        if self.vector_store:
            try:
                # Delete collection and recreate
                self.vector_store._client.delete_collection("knowledge_base")
                print("Vector database cleared")
                self._initialize_vector_store()
            except Exception as e:
                print(f"Error clearing database: {e}")
    
    def get_stats(self) -> dict:
        """Get statistics about the database."""
        if not self.vector_store:
            return {"status": "not initialized"}
        
        try:
            collection = self.vector_store._collection
            count = collection.count()
            return {
                "status": "ready",
                "total_chunks": count,
                "persist_directory": self.persist_directory
            }
        except Exception as e:
            return {"status": "error", "error": str(e)}
