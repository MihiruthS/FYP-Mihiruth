"""
Robot Voice Pipeline - Main Orchestrator

A complete voice pipeline for robots: Listen → Think → Answer → Loop

This pipeline:
1. Listens to user speech via microphone
2. Transcribes speech to text using AssemblyAI
3. Processes the query using an AI agent (OpenAI GPT)
4. Synthesizes a speech response using Cartesia TTS
5. Plays the response through speakers
6. Returns to listening for the next query

Usage:
    python main.py
"""

import asyncio
import os
import sys
from pathlib import Path
from dotenv import load_dotenv

from robot_pipeline.audio.capture import AudioCapture
from robot_pipeline.audio.playback import AudioPlayback
from robot_pipeline.speech.stt import SpeechToText
from robot_pipeline.speech.tts import TextToSpeech
from robot_pipeline.ai.agent import AIAgent
from robot_pipeline.ai.prompts import PromptGenerator
from robot_pipeline.ai.rag_database import RAGDatabase
from robot_pipeline.ai.faq_database import FAQDatabase

# Get the project root directory (parent of src/)
PROJECT_ROOT = Path(__file__).parent.parent.parent.absolute()


class RobotVoicePipeline:
    """
    Complete voice interaction pipeline for robots.
    
    Orchestrates the entire flow: listen → transcribe → think → speak
    """
    
    def __init__(self):
        """
        Initialize the robot voice pipeline with PromptGenerator and RAG Database.
        """
        # Initialize prompt generator
        self.prompt_generator = PromptGenerator()
        
        # Initialize FAQ database (fast responses)
        print("⚡ Initializing FAQ Database...")
        self.faq_database = None
        try:
            self.faq_database = FAQDatabase()
            if self.faq_database and self.faq_database.faqs:
                faq_stats = self.faq_database.get_stats()
                print(f"✅ FAQ Database ready with {faq_stats['total_faqs']} questions")
            else:
                print("⚠️  FAQ Database initialized but empty")
                self.faq_database = None
        except Exception as e:
            print(f"⚠️  FAQ Database initialization failed: {e}")
            self.faq_database = None
        
        # Initialize RAG database with absolute paths
        print("🔍 Initializing RAG Database...")
        chroma_db_path = PROJECT_ROOT / "data" / "chroma_db"
        docs_path = PROJECT_ROOT / "src" / "knowledge_base" / "documents"
        
        print(f"   Database: {chroma_db_path}")
        print(f"   Documents: {docs_path}")
        
        self.rag_database = RAGDatabase(
            persist_directory=str(chroma_db_path),
            documents_directory=str(docs_path)
        )
        
        # Display database stats
        stats = self.rag_database.get_stats()
        if stats.get("status") == "ready":
            print(f"✅ RAG Database ready with {stats.get('total_chunks', 0)} knowledge chunks")
        
        # Initialize components
        self.audio_capture = AudioCapture(sample_rate=16000)
        self.audio_playback = AudioPlayback(sample_rate=24000)
        self.stt = SpeechToText()
        self.tts = TextToSpeech()
        self.agent = AIAgent(
            prompt_generator=self.prompt_generator,
            rag_database=self.rag_database,
            faq_database=self.faq_database,
            use_rag=True,
            use_faq=True
        )
        
        print("🤖 Robot Voice Pipeline Initialized with RAG")
    
    async def process_query(self) -> bool:
        """
        Process a single query: listen → transcribe → think → speak.
        Uses VAD (Voice Activity Detection) - no fixed duration.
        Streams TTS in real-time as LLM generates sentences.
        
        Returns:
            True to continue, False to stop the pipeline
        """
        print("\n" + "="*60)
        print("👂 LISTENING... (speak now, I'll detect when you stop)")
        print("="*60)
        
        # Start audio capture
        self.audio_capture.start()
        
        # Create stop event for VAD
        stop_audio_event = asyncio.Event()
        
        # Create audio stream without duration limit (VAD will handle it)
        audio_stream = self.audio_capture.stream(stop_event=stop_audio_event)
        
        try:
            # Transcribe audio stream with VAD
            transcript = None
            
            # This will automatically stop when AssemblyAI detects speech end
            async for text in self.stt.transcribe_stream(audio_stream):
                transcript = text
                # Signal audio capture to stop
                stop_audio_event.set()
                print(f"📝 You said: '{text}'")
            
            # Stop audio capture
            self.audio_capture.stop()
            
            # Check if we got a transcript
            if not transcript or not transcript.strip():
                print("❌ No speech detected. Try again.")
                return True
            
            # Check for exit commands
            if transcript.lower().strip() in ["exit", "quit", "stop", "goodbye", "bye"]:
                print("👋 Goodbye!")
                return False
            
            # Think and Speak: Stream response with real-time TTS
            # Ensure TTS and audio are ready (will skip if already connected)
            if not self.tts._is_connected:
                await self.tts.connect()
            if not self.audio_playback._is_playing:
                self.audio_playback.start()
            
            print(f"🤔 Processing...")
            
            # Buffer for phrase detection - use smaller chunks for faster initial audio
            text_buffer = ""
            # Detect shorter phrase boundaries for faster TTS start
            phrase_endings = [". ", "! ", "? ", ", ", ".\n", "!\n", "?\n"]
            sentence_endings = [". ", "! ", "? ", ".\n", "!\n", "?\n"]
            first_audio = True
            min_phrase_length = 10  # Start TTS after at least 10 chars
            
            # Stream AI response and synthesize phrases in real-time
            async for chunk in self.agent.think_stream(transcript):
                text_buffer += chunk
                
                # Check if we have a complete phrase (prioritize sentences, then commas)
                should_synthesize = False
                phrase = ""
                
                # First check for sentence endings (higher priority)
                for ending in sentence_endings:
                    if ending in text_buffer and len(text_buffer) >= min_phrase_length:
                        parts = text_buffer.split(ending, 1)
                        phrase = parts[0] + ending.strip()
                        text_buffer = parts[1] if len(parts) > 1 else ""
                        should_synthesize = True
                        break
                
                # If no sentence, check for phrase boundaries (commas) for faster start
                if not should_synthesize and len(text_buffer) >= 30:  # Only for longer phrases
                    for ending in [", "]:
                        if ending in text_buffer:
                            parts = text_buffer.split(ending, 1)
                            phrase = parts[0] + ending
                            text_buffer = parts[1] if len(parts) > 1 else ""
                            should_synthesize = True
                            break
                
                # Synthesize and play this phrase immediately
                if should_synthesize and phrase.strip():
                    if first_audio:
                        print(f"🗣️ Speaking...")
                        first_audio = False
                    
                    # Stream this phrase to TTS and play (connection already open)
                    audio_stream = self.tts.synthesize_stream(phrase)
                    await self.audio_playback.play_stream(audio_stream)
            
            # Handle any remaining text in buffer (incomplete sentence)
            if text_buffer.strip():
                audio_stream = self.tts.synthesize_stream(text_buffer)
                await self.audio_playback.play_stream(audio_stream)
            
            print("✅ Response complete")
            
            return True
            
        except KeyboardInterrupt:
            print("\n\n⚠️ Interrupted by user")
            return False
        except Exception as e:
            print(f"❌ Error processing query: {e}")
            import traceback
            traceback.print_exc()
            return True
    
    async def run(self):
        """Run the voice pipeline in a continuous loop."""
        print("\n" + "="*60)
        print("🤖 ROBOT VOICE PIPELINE STARTED")
        print("="*60)
        print("🎙️  Voice Activity Detection (VAD) enabled - speak naturally")
        print("⚡ Real-time streaming: TTS starts while AI is thinking")
        print("🛑 Say 'exit', 'quit', or 'stop' to end the session")
        print("⌨️  Press Ctrl+C to force quit")
        print("="*60)
        
        # Pre-connect to services for lower latency
        print("🔗 Connecting to services...")
        try:
            await self.tts.connect()
            self.audio_playback.start()
            print("✅ Ready!")
        except Exception as e:
            print(f"⚠️ Could not pre-connect: {e}")
        
        try:
            # Main loop: listen → think → answer → repeat
            while True:
                should_continue = await self.process_query()
                
                if not should_continue:
                    break
                
                # Small pause between queries
                await asyncio.sleep(0.3)
        
        except KeyboardInterrupt:
            print("\n\n⚠️ Pipeline interrupted by user")
        
        finally:
            # Cleanup
            print("\n🧹 Cleaning up...")
            self.audio_capture.stop()
            self.audio_playback.stop()
            await self.stt.close()
            await self.tts.close()
            print("✅ Pipeline stopped")


async def main():
    """Main entry point."""
    # Load environment variables
    load_dotenv()
    
    # Check required API keys
    required_keys = ["ASSEMBLYAI_API_KEY", "CARTESIA_API_KEY", "OPENAI_API_KEY"]
    missing_keys = [key for key in required_keys if not os.getenv(key)]
    
    if missing_keys:
        print("❌ Missing required API keys:")
        for key in missing_keys:
            print(f"   - {key}")
        print("\nPlease set these in your .env file or environment variables.")
        sys.exit(1)
    
    # Create and run pipeline with PromptGenerator
    pipeline = RobotVoicePipeline()
    
    await pipeline.run()


if __name__ == "__main__":
    # Run the async main function
    asyncio.run(main())
