# 🤖 Robot Voice Pipeline

A complete voice interaction pipeline for humanoid robots with RAG (Retrieval-Augmented Generation) capabilities.

## 🎯 Overview

This pipeline enables natural voice conversations with robots by combining:
- 🎤 **Speech Recognition** (AssemblyAI)
- 🧠 **AI Agent** with RAG (OpenAI GPT-4o-mini + ChromaDB)
- 🔊 **Speech Synthesis** (Cartesia TTS)
- 📚 **Knowledge Base** for domain-specific answers

## 🚀 Quick Start

### 1. Setup

```bash
# Clone or navigate to project
cd robot-voice-pipeline

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -e .

# Configure environment variables
cp .env.example .env
# Edit .env and add your API keys
```

### 2. Initialize Knowledge Base

```bash
# Index documents into vector database
python scripts/init_database.py
```

### 3. Run the Pipeline

```bash
# Start voice interaction
python main.py
```

## 📁 Project Structure

```
robot-voice-pipeline/
├── src/
│   ├── robot_pipeline/           # Main package
│   │   ├── audio/                # Audio capture & playback
│   │   ├── speech/               # STT & TTS
│   │   ├── ai/                   # AI agent, prompts, RAG
│   │   └── pipeline.py           # Main orchestrator
│   └── knowledge_base/
│       └── documents/            # Knowledge documents (.txt)
├── scripts/
│   ├── init_database.py          # Initialize RAG database
│   ├── test_rag.py               # Test RAG integration
│   └── list_voices.py            # List available TTS voices
├── tests/                         # Test files
├── docs/                          # Documentation
├── data/
│   └── chroma_db/                # Vector database
├── main.py                        # Entry point
└── pyproject.toml                 # Dependencies
```

## 🎓 Features

- ✅ Real-time speech recognition with VAD
- ✅ RAG-powered knowledge retrieval
- ✅ Natural language understanding
- ✅ High-quality speech synthesis
- ✅ Department information & guidance
- ✅ Escort services to locations

## 🛠️ Usage

### Run Voice Pipeline

```bash
python main.py
```

### Manage Knowledge Base

```bash
# View database stats
python scripts/init_database.py

# Reset and rebuild
python scripts/init_database.py --reset

# Test RAG
python scripts/test_rag.py
```

### Add New Documents

1. Add `.txt` files to `src/knowledge_base/documents/`
2. Rebuild: `python scripts/init_database.py --reset`

## 📚 Documentation

- [RAG Guide](docs/RAG_GUIDE.md)
- [Architecture](docs/ARCHITECTURE.md)
- [Integration Summary](docs/RAG_INTEGRATION_SUMMARY.md)

## 🐛 Troubleshooting

**RAG not working?**
```bash
python scripts/init_database.py --reset
```

**Import errors?**
```bash
source venv/bin/activate
pip install -e .
```

---

**Built for**: Electronic & Telecommunication Engineering Department, University of Moratuwa  
**Version**: 0.1.0  
**Status**: Production Ready ✅
