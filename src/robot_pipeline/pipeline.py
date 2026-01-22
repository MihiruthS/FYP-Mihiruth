"""
Robot Voice Pipeline - Main Orchestrator

A complete voice pipeline for robots: Listen → Think → Answer → Loop
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


PROJECT_ROOT = Path(__file__).parent.parent.parent.absolute()


class RobotVoicePipeline:
    def __init__(self):
        self.is_awake = False
        # Wake words with common STT mishearings of "Quanta"
        self.wake_words = [
            "hey quanta", "hello quanta", "hi quanta",
            "hey conda", "hello conda", "hi conda",
            "hey konda", "hello konda", "hi konda",
            "hey konta", "hello konta", "hi konta",
            "hey quantum", "hello quantum", "hi quantum",
            "hey pointer", "hello pointer", "hi pointer",
            "hey fonda", "hello fonda", "hi fonda",
            "hey enter", "hello enter", "hi enter",
            "he enter", "hey counter", "hello counter",
            "hey panda", "hello panda", "hi panda",
            "hey honda", "hello honda", "hi honda",
            "hey hong", "hello hong", "hi hong",
            "hey contact", "hello contact", "hi contact",
            "hey quota", "hello quota", "hi quota"
        ]
        self.sleep_words = ["exit", "quit", "stop", "goodbye", "bye", "thank you", "thanks"]

        self.prompt_generator = PromptGenerator()

        print("Initializing FAQ Database...")
        try:
            self.faq_database = FAQDatabase()
            stats = self.faq_database.get_stats()
            print(f"FAQ Database ready with {stats['total_faqs']} questions")
        except Exception as e:
            print(f"FAQ Database failed: {e}")
            self.faq_database = None

        print("Initializing RAG Database...")
        self.rag_database = RAGDatabase(
            persist_directory=str(PROJECT_ROOT / "data" / "chroma_db"),
            documents_directory=str(PROJECT_ROOT / "src" / "knowledge_base" / "documents")
        )

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

        print("Robot Voice Pipeline Initialized")

    def _clean_text(self, text: str) -> str:
        """Remove punctuation and normalize text for matching."""
        import string
        # Remove punctuation and convert to lowercase
        text = text.lower().strip()
        text = text.translate(str.maketrans('', '', string.punctuation))
        return text

    async def listen_for_wake_word(self) -> bool:
        print("\nSleeping... Say 'Hey Quanta'")

        self.audio_capture.start()
        stop_event = asyncio.Event()
        audio_stream = self.audio_capture.stream(stop_event=stop_event)

        try:
            transcript = None
            async for text in self.stt.transcribe_stream(audio_stream):
                transcript = text
                stop_event.set()
                print(f"Heard: {text}")

            self.audio_capture.stop()

            if not transcript:
                return True

            # Clean text by removing punctuation for better matching
            text = self._clean_text(transcript)

            if any(w in text for w in self.wake_words):
                self.is_awake = True
                await self._speak("Hello! How can I help you?")
                return True

            # Also check original text for sleep words (with punctuation)
            if any(sleep_word in transcript.lower() for sleep_word in self.sleep_words):
                return False

            return True

        except KeyboardInterrupt:
            return False

    async def process_query(self) -> bool:
        print("\nListening...")

        self.audio_capture.start()
        stop_event = asyncio.Event()
        audio_stream = self.audio_capture.stream(stop_event=stop_event)

        try:
            transcript = None
            async for text in self.stt.transcribe_stream(audio_stream):
                transcript = text
                stop_event.set()
                print(f"You said: {text}")

            self.audio_capture.stop()

            if not transcript:
                return True

            # Smart sleep word detection - only sleep if it's clearly a goodbye
            # Not if "thank you" is part of a longer sentence with a question
            text_cleaned = self._clean_text(transcript)
            text_lower = transcript.lower()
            
            # Check if the message is ONLY a sleep word (with minimal extra words)
            # Split into sentences by period, question mark, exclamation
            import re
            sentences = re.split(r'[.!?]+', text_lower)
            sentences = [s.strip() for s in sentences if s.strip()]
            
            # If there's only one short sentence and it contains a sleep word, go to sleep
            if len(sentences) == 1 and len(text_cleaned.split()) <= 3:
                if any(sleep_word in text_lower for sleep_word in self.sleep_words):
                    self.is_awake = False
                    await self._speak("Have a nice day")
                    return True
            
            # Also check if the last sentence is ONLY a goodbye (handles "answer. Thank you!")
            if len(sentences) > 1:
                last_sentence = sentences[-1].strip()
                if any(sleep_word == last_sentence for sleep_word in self.sleep_words):
                    self.is_awake = False
                    await self._speak("Going to sleep. Say Hey Quanta when you need me.")
                    return True

            await self._stream_response(transcript)
            return True

        except KeyboardInterrupt:
            return False

    async def _stream_response(self, user_text: str):
        if not self.tts._is_connected:
            await self.tts.connect()
        if not self.audio_playback._is_playing:
            self.audio_playback.start()

        buffer = ""
        async for chunk in self.agent.think_stream(user_text):
            buffer += chunk
            
            # Split buffer at punctuation boundaries
            while True:
                # Find the earliest punctuation mark followed by space
                split_pos = -1
                for punct in [". ", "! ", "? ", ", "]:
                    pos = buffer.find(punct)
                    if pos != -1 and (split_pos == -1 or pos < split_pos):
                        split_pos = pos + len(punct)
                
                # If we found a complete sentence/phrase, synthesize it
                if split_pos > 0:
                    sentence = buffer[:split_pos].strip()
                    if sentence:
                        audio = self.tts.synthesize_stream(sentence)
                        await self.audio_playback.play_stream(audio)
                    buffer = buffer[split_pos:]
                else:
                    break

        # Synthesize any remaining text
        if buffer.strip():
            audio = self.tts.synthesize_stream(buffer)
            await self.audio_playback.play_stream(audio)

    async def _speak(self, text: str):
        if not self.tts._is_connected:
            await self.tts.connect()
        if not self.audio_playback._is_playing:
            self.audio_playback.start()

        audio = self.tts.synthesize_stream(text)
        await self.audio_playback.play_stream(audio)

    async def run(self):
        print("\nROBOT VOICE PIPELINE STARTED")

        try:
            await self.tts.connect()
            self.audio_playback.start()
        except Exception as e:
            print(f"Pre-connect failed: {e}")

        try:
            while True:
                if not self.is_awake:
                    if not await self.listen_for_wake_word():
                        break
                else:
                    if not await self.process_query():
                        break

                await asyncio.sleep(0.2)

        finally:
            print("\nCleaning up...")
            self.audio_capture.stop()
            self.audio_playback.stop()
            await self.stt.close()
            await self.tts.close()
            print("Stopped")


async def main():
    load_dotenv()

    required = ["ASSEMBLYAI_API_KEY", "CARTESIA_API_KEY", "OPENAI_API_KEY"]
    missing = [k for k in required if not os.getenv(k)]

    if missing:
        print("Missing API keys:")
        for k in missing:
            print(f" - {k}")
        sys.exit(1)

    pipeline = RobotVoicePipeline()
    await pipeline.run()


if __name__ == "__main__":
    asyncio.run(main())
