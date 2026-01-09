# RAG Integration Guide

This document explains the Retrieval-Augmented Generation (RAG) integration in the Robot Voice Pipeline.

## Overview

RAG enables your robot to answer questions using knowledge from documents stored in `src/vector_database_documents/`. When a user asks a question, the system:

1. **Retrieves** relevant information from the knowledge base using semantic search
2. **Augments** the AI prompt with retrieved context
3. **Generates** an accurate response based on the retrieved knowledge

## Architecture

```
User Query → RAG Database (ChromaDB) → Retrieve Top-K Documents → AI Agent → Response
```

### Components

- **rag_database.py**: Manages vector storage, embeddings, and retrieval
- **ChromaDB**: Vector database for storing document embeddings
- **OpenAI Embeddings**: `text-embedding-3-small` model for semantic search
- **Document Chunking**: Splits documents into 1000-character chunks with 200-character overlap

## Setup Instructions

### 1. Install Dependencies

```bash
# Install RAG-related packages
pip install langchain-chroma chromadb langchain-text-splitters

# Or use uv (recommended)
uv pip install langchain-chroma chromadb langchain-text-splitters
```

### 2. Initialize the Database

Run the initialization script to load and index your documents:

```bash
python init_rag_database.py
```

Options:
- `--reset`: Clear existing database and rebuild from scratch
- `--docs-dir`: Path to documents directory (default: `./src/vector_database_documents`)
- `--db-dir`: Path to ChromaDB directory (default: `./chroma_db`)

Example:
```bash
# Reset and rebuild database
python init_rag_database.py --reset

# Use custom directories
python init_rag_database.py --docs-dir ./my_docs --db-dir ./my_db
```

### 3. Run the Pipeline

The pipeline will automatically use RAG when initialized:

```bash
python main.py
```

## Adding New Documents

1. Add `.txt` files to `src/vector_database_documents/`
2. Run the initialization script with `--reset`:
   ```bash
   python init_rag_database.py --reset
   ```

## Document Format

Documents should be plain text (`.txt`) files. The system will:
- Automatically split long documents into chunks
- Preserve source file information in metadata
- Create embeddings for semantic search

Example document structure:
```
ENTC_Introduction.txt
Academic_Programs.txt
Department_Locations.txt
```

## How It Works

### 1. Document Loading & Indexing

```python
# Documents are loaded, split, and embedded
documents = load_documents("src/vector_database_documents/")
chunks = split_documents(documents, chunk_size=1000, overlap=200)
embeddings = create_embeddings(chunks)
store_in_chromadb(embeddings)
```

### 2. Query Processing

```python
# When user asks a question:
user_query = "Who is the head of department?"

# Retrieve relevant chunks
relevant_docs = retrieve(user_query, k=3)

# Add to AI prompt
context = format_context(relevant_docs)
prompt = build_prompt(user_query, context)

# Generate response
response = ai_agent.generate(prompt)
```

### 3. Context Integration

Retrieved context is automatically added to the AI prompt:

```
RETRIEVED KNOWLEDGE:
[Source: Department_Staff.txt]
The Head of Department is Dr. Thayaparan Subramaniam...

---

[Source: ENTC_Introduction.txt]
The department offers undergraduate and postgraduate programs...

USE THE ABOVE INFORMATION TO ANSWER QUESTIONS ACCURATELY.
```

## Configuration

### RAG Parameters (in `rag_database.py`)

```python
RAGDatabase(
    persist_directory="./chroma_db",        # Where to store the database
    documents_directory="./src/vector_database_documents",  # Document folder
    chunk_size=1000,                        # Size of text chunks
    chunk_overlap=200,                      # Overlap between chunks
)
```

### Retrieval Parameters (in `ai_agent.py`)

```python
context = rag_database.get_context_string(
    query=user_input,
    k=3,                    # Number of chunks to retrieve
    max_length=2000         # Maximum context length
)
```

## Troubleshooting

### Database Not Found

If you see "Empty vector database detected":
```bash
python init_rag_database.py
```

### OpenAI API Key Missing

Ensure your `.env` file contains:
```
OPENAI_API_KEY=your-api-key-here
```

### No Documents Retrieved

1. Check if documents exist in `src/vector_database_documents/`
2. Verify database has been initialized: `python init_rag_database.py --reset`
3. Test retrieval with the initialization script

### Import Errors

```bash
# Install missing packages
pip install langchain-chroma chromadb langchain-text-splitters
```

## Performance Tips

1. **Chunk Size**: Larger chunks (1000-1500) for detailed info, smaller (500-800) for quick facts
2. **Number of Chunks (k)**: Start with 3, increase to 5 for complex queries
3. **Max Context Length**: Keep under 2000 tokens for faster responses
4. **Embedding Model**: `text-embedding-3-small` is cost-effective; upgrade to `text-embedding-3-large` for better accuracy

## API Costs

Using OpenAI embeddings:
- **text-embedding-3-small**: ~$0.00002 per 1K tokens
- Example: Indexing 10 documents (~50K tokens) ≈ $0.001

Cost is one-time during indexing. Queries use cached embeddings.

## Disabling RAG

To run without RAG, modify `main.py`:

```python
self.agent = AIAgent(
    prompt_generator=self.prompt_generator,
    rag_database=None,
    use_rag=False
)
```

## Advanced Usage

### Programmatic Access

```python
from rag_database import RAGDatabase

# Initialize
rag = RAGDatabase(
    persist_directory="./chroma_db",
    documents_directory="./src/vector_database_documents"
)

# Retrieve documents
docs = rag.retrieve("What programs are offered?", k=3)

# Get formatted context
context = rag.get_context_string("Who is the head?", k=2)

# Get statistics
stats = rag.get_stats()
print(f"Total chunks: {stats['total_chunks']}")
```

### Custom Filtering

You can filter by metadata (e.g., source file):

```python
results = rag.vector_store.similarity_search(
    query="programs",
    k=5,
    filter={"source": "Academic_Programs.txt"}
)
```

## File Structure

```
robot-voice-pipeline/
├── src/
│   ├── rag_database.py           # RAG implementation
│   ├── ai_agent.py                # Updated with RAG integration
│   ├── prompt_templates.py        # Updated with context handling
│   └── vector_database_documents/ # Knowledge base
│       ├── ENTC_Introduction.txt
│       ├── Academic_Programs.txt
│       └── ...
├── chroma_db/                     # Vector database (auto-created)
├── init_rag_database.py           # Initialization script
└── README_RAG.md                  # This file
```

## Next Steps

1. ✅ Install dependencies
2. ✅ Add your documents to `src/vector_database_documents/`
3. ✅ Run `python init_rag_database.py`
4. ✅ Test with `python main.py`
5. 🚀 Enjoy knowledge-powered conversations!

## Support

For issues or questions:
1. Check the troubleshooting section above
2. Verify all dependencies are installed
3. Ensure OpenAI API key is configured
4. Review error messages in the console

---

**Built with:**
- LangChain for RAG orchestration
- ChromaDB for vector storage
- OpenAI for embeddings and LLM
