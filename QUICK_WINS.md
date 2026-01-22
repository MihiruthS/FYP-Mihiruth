# 🚀 Quick Performance Wins (Not Yet Implemented)

These are simple changes you can make for additional speedups:

---

## 1. Switch to GPT-3.5-Turbo (2-3x Faster) ⭐⭐⭐

**Change one line in agent.py:**

```python
# In src/robot_pipeline/ai/agent.py, line ~73
self.llm = ChatOpenAI(
    model="gpt-3.5-turbo",  # Change from "gpt-4o-mini"
    api_key=self.api_key,
    temperature=0.1,
    streaming=True,
    max_tokens=60,
)
```

**Impact:** 2-3x faster LLM responses, 50% cost savings  
**Trade-off:** Slightly lower quality (test first!)

---

## 2. Pre-warm Connections (0.5-1s Faster First Query) ⭐⭐

**In pipeline.py, add to __init__:**

```python
# After all initializations in __init__, add:
print("🔥 Pre-warming connections...")
asyncio.create_task(self._prewarm_connections())
```

**Add new method:**

```python
async def _prewarm_connections(self):
    """Pre-connect to TTS/STT to eliminate first-query delay."""
    try:
        await self.tts.connect()
        await self.stt.connect()
        print("✅ Connections pre-warmed")
    except Exception as e:
        print(f"⚠️ Pre-warm failed: {e}")
```

**Impact:** First interaction 0.5-1s faster  
**Trade-off:** Slightly longer startup (but happens async)

---

## 3. Reduce FAQ Similarity Threshold (More FAQ Hits) ⭐⭐

**In faq_database.py:**

```python
# Line ~51, change:
similarity_threshold: float = 0.78,  # Lower from 0.82
```

**Impact:** More queries match FAQs = faster responses  
**Trade-off:** May match less similar questions (test carefully!)

---

## 4. Disable Web Search (If Not Needed) ⭐

**In pipeline.py, line ~81:**

```python
self.agent = AIAgent(
    prompt_generator=self.prompt_generator,
    rag_database=self.rag_database,
    faq_database=self.faq_database,
    use_rag=True,
    use_faq=True,
    use_web_search=False  # Change from True
)
```

**Impact:** Eliminates 1-2s for queries that would trigger web search  
**Trade-off:** Can't answer current events/world questions

---

## 5. Increase TTS Speed to "fast" ⭐

**In pipeline.py or tts.py:**

```python
self.tts = TextToSpeech(speed="fast")  # Change from "normal"
```

**Impact:** 20-30% faster audio generation  
**Trade-off:** Might sound rushed, test if acceptable

---

## 6. Reduce RAG Documents (Faster Startup) ⭐

**Remove unused documents from:**
```
src/knowledge_base/documents/
```

**Impact:** Faster startup, smaller vector DB  
**Trade-off:** Less knowledge available

---

## 7. Batch FAQ Embeddings (Faster First Run) ⭐⭐

**In faq_database.py, _compute_embeddings method:**

Current code already computes all at once - no change needed! ✅

---

## 8. Use Lighter Embedding Model ⭐

**In faq_database.py and rag_database.py:**

```python
self.embeddings_model = OpenAIEmbeddings(
    api_key=self.api_key,
    model="text-embedding-3-small"  # Already using smallest!
)
```

Already optimized! ✅

---

## Recommended Order of Implementation

1. ✅ **Pre-warm connections** - Easy, no downsides
2. ⚠️ **Test GPT-3.5-turbo** - Huge speedup if quality is OK
3. ⚠️ **Lower FAQ threshold** - Test carefully for accuracy
4. ⚠️ **Increase TTS speed** - Test if users prefer it
5. ❌ **Disable web search** - Only if truly not needed

---

## Quick Test Commands

```bash
# Test current performance
time python -c "from src.robot_pipeline.ai.faq_database import FAQDatabase; f = FAQDatabase()"

# Measure response time
python scripts/test_performance.py
```

---

**Want even MORE speed?** Consider local models:
- Whisper (local STT): ~100ms latency
- Piper (local TTS): ~50ms latency
- But requires GPU and more complex setup
