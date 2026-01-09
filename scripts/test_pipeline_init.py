"""Test pipeline initialization to debug FAQ."""
import sys
from pathlib import Path

# Add src to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root / "src"))

from dotenv import load_dotenv
load_dotenv()

print("="*60)
print("Testing Pipeline Initialization")
print("="*60)

# Test FAQ first
print("\n1️⃣ Testing FAQ Database directly:")
from robot_pipeline.ai.faq_database import FAQDatabase
faq = FAQDatabase()
print(f"   FAQ object: {faq}")
print(f"   Has FAQs: {len(faq.faqs) if faq.faqs else 0}")
print(f"   Embeddings model: {faq.embeddings_model}")

# Test pipeline
print("\n2️⃣ Testing Pipeline Initialization:")
from robot_pipeline.pipeline import RobotVoicePipeline

# Patch to see what happens
class TestPipeline(RobotVoicePipeline):
    def __init__(self):
        print("   Starting pipeline init...")
        super().__init__()
        print(f"   FAQ database after init: {self.faq_database}")
        if self.faq_database:
            print(f"   FAQ has faqs: {len(self.faq_database.faqs)}")
        print(f"   Agent.use_faq: {self.agent.use_faq}")
        print(f"   Agent.faq_database: {self.agent.faq_database}")

pipeline = TestPipeline()

print("\n3️⃣ Testing FAQ query through agent:")
import asyncio
async def test():
    response = await pipeline.agent.think("Who is the head of the department?")
    print(f"   Response: {response}")

asyncio.run(test())

print("\n" + "="*60)
print("Test Complete")
print("="*60)
