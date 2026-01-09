# RAG Architecture Overview

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    ROBOT VOICE PIPELINE                         │
│                         with RAG                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────┐      ┌──────────────┐      ┌─────────────┐
│   Microphone│─────▶│ Audio Capture│─────▶│ Speech-to-  │
│   (Input)   │      │   (16kHz)    │      │    Text     │
└─────────────┘      └──────────────┘      │(AssemblyAI) │
                                            └──────┬──────┘
                                                   │
                                                   ▼
                                          ┌────────────────┐
                                          │  User Query    │
                                          │  "Who is HOD?" │
                                          └────────┬───────┘
                                                   │
                        ┌──────────────────────────┴──────────────────────┐
                        │                                                  │
                        ▼                                                  ▼
            ┌───────────────────────┐                        ┌────────────────────┐
            │   RAG Database        │                        │   Prompt Generator │
            │   (ChromaDB)          │                        │  (System Prompt)   │
            │                       │                        └────────────────────┘
            │  • 61 chunks          │                                  │
            │  • OpenAI Embeddings  │                                  │
            │  • Semantic Search    │                                  │
            └──────────┬────────────┘                                  │
                       │                                                │
                       │ Retrieve Top-3 Chunks                         │
                       │                                                │
                       ▼                                                │
            ┌───────────────────────┐                                  │
            │  Context:             │                                  │
            │  "HOD is Dr.          │                                  │
            │   Thayaparan..."      │                                  │
            └──────────┬────────────┘                                  │
                       │                                                │
                       └───────────────────┬────────────────────────────┘
                                           │
                                           ▼
                                  ┌────────────────┐
                                  │   AI Agent     │
                                  │ (GPT-4o-mini)  │
                                  │                │
                                  │ • Uses Context │
                                  │ • Generates    │
                                  │   Response     │
                                  └────────┬───────┘
                                           │
                                           ▼
                                  ┌────────────────┐
                                  │    Response    │
                                  │ "The HOD is    │
                                  │ Dr. Thayaparan │
                                  │  Subramaniam." │
                                  └────────┬───────┘
                                           │
                                           ▼
                                  ┌────────────────┐
                                  │ Text-to-Speech │
                                  │  (Cartesia)    │
                                  └────────┬───────┘
                                           │
                                           ▼
                                  ┌────────────────┐
                                  │ Audio Playback │
                                  │   (Speakers)   │
                                  └────────────────┘
```

## Data Flow

### 1. Document Indexing (One-time Setup)

```
Documents (.txt files)
    │
    ├─▶ Load from src/vector_database_documents/
    │
    ├─▶ Split into chunks (1000 chars, 200 overlap)
    │
    ├─▶ Generate embeddings (OpenAI text-embedding-3-small)
    │
    └─▶ Store in ChromaDB (./chroma_db/)
```

### 2. Query Processing (Runtime)

```
User Query
    │
    ├─▶ Generate query embedding
    │
    ├─▶ Semantic search in ChromaDB
    │
    ├─▶ Retrieve top-K similar chunks (K=3)
    │
    ├─▶ Format as context string
    │
    └─▶ Inject into AI prompt
         │
         ├─▶ System Prompt (Robot Profile)
         ├─▶ Retrieved Context (RAG)
         ├─▶ Chat History
         └─▶ User Query
              │
              └─▶ Generate Response
```

## Components

### Core Modules

| Module | Purpose | Technology |
|--------|---------|------------|
| **rag_database.py** | Vector DB management | ChromaDB, LangChain |
| **ai_agent.py** | LLM orchestration | OpenAI GPT-4o-mini |
| **prompt_templates.py** | Prompt engineering | Custom templates |
| **main.py** | Pipeline orchestration | AsyncIO |

### Storage

```
robot-voice-pipeline/
├── src/
│   └── vector_database_documents/    # Source documents (9 files)
│       ├── ENTC_Introduction.txt
│       ├── Academic_Programs.txt
│       └── ...
│
└── chroma_db/                         # Vector database
    ├── chroma.sqlite3                 # Metadata
    └── [embedding vectors]            # Stored embeddings
```

## RAG Pipeline Details

### Embedding Generation

```python
# Using OpenAI text-embedding-3-small
Document → Chunks → Embeddings (1536 dimensions)
                        ↓
                   ChromaDB Storage
```

### Retrieval Process

```python
Query → Embedding → Cosine Similarity → Top-K Chunks
                                             ↓
                                    Formatted Context
```

### Context Injection

```python
System Prompt = f"""
    ROBOT PROFILE: {robot_profile}
    
    RETRIEVED KNOWLEDGE:
    {rag_context}  # ← Injected from ChromaDB
    
    RULES: {rules}
    
    USER QUERY: {user_input}
"""
```

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **Documents** | 9 | .txt files |
| **Total Chunks** | 61 | 1000 chars each |
| **Embedding Dim** | 1536 | OpenAI model |
| **Retrieval Time** | ~50-100ms | Per query |
| **Accuracy Gain** | High | For domain questions |
| **Storage** | ~5MB | ChromaDB |

## Configuration Options

### RAG Parameters

```python
# Chunking
chunk_size = 1000          # Characters per chunk
chunk_overlap = 200        # Overlap for context

# Retrieval
k = 3                      # Number of chunks to retrieve
max_context_length = 2000  # Max chars in context

# Embeddings
model = "text-embedding-3-small"  # OpenAI model
```

### Toggle RAG

```python
# Enable RAG (default)
agent = AIAgent(rag_database=rag_db, use_rag=True)

# Disable RAG
agent = AIAgent(rag_database=None, use_rag=False)
```

## Dependencies

```toml
langchain-chroma      # ChromaDB integration
chromadb              # Vector database
langchain-text-splitters  # Document chunking
langchain-openai      # OpenAI integration
```

## API Calls

### Initialization (One-time)
- **OpenAI Embeddings API**: 61 chunks × ~250 tokens = ~15,250 tokens
- **Cost**: ~$0.001

### Per Query (Runtime)
- **OpenAI Embeddings API**: 1 query embedding (~10-50 tokens)
- **OpenAI Chat API**: 1 completion (prompt + response)
- **Cost**: ~$0.0001-0.0005 per query

## Error Handling

```python
try:
    # Retrieve context
    context = rag_db.get_context_string(query)
except Exception as e:
    # Fallback to no context
    context = ""
    print(f"RAG retrieval failed: {e}")

# AI agent proceeds with or without context
response = agent.think(query)
```

## Future Enhancements

- [ ] Add metadata filtering (by document type, date, etc.)
- [ ] Implement hybrid search (keyword + semantic)
- [ ] Add document update tracking
- [ ] Support for PDF/DOCX documents
- [ ] Multi-language support
- [ ] Citation tracking (which document answered)
- [ ] Real-time document updates without restart

---

**Built with:**
- 🧠 LangChain for RAG orchestration
- 📚 ChromaDB for vector storage
- 🤖 OpenAI for embeddings & LLM
- 🎙️ AssemblyAI for speech-to-text
- 🔊 Cartesia for text-to-speech
