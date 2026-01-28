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
        self.is_escorting = False  # Track if robot is currently escorting
        self.pending_escort = None  # Tracks location waiting for confirmation
        self.last_mentioned_location = None  # Track last location mentioned in conversation
        
        # Location mapping for escort commands
        self.location_mapping = {
            "computer lab": "computer_lab",
            "conference room": "conference_room",
            "head of the department's office": "hod_office",
            "head of department's office": "hod_office",
            "hod office": "hod_office",
            "department office": "department_office",
        }
        
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
        self.confirmation_words = ["yes", "sure", "okay", "ok", "please", "confirm", "yeah", "yep"]
        self.rejection_words = ["no", "nope", "cancel", "nevermind", "never mind"]

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
        self.audio_playback = AudioPlayback(sample_rate=24000)  # mouth_controller added later via ROS2
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
    
    def _check_escort_request(self, response: str):
        """Check if response is asking for escort confirmation."""
        response_lower = response.lower()
        
        # Pattern 1: Direct confirmation request ("PLEASE CONFIRM...")
        # Pattern 2: Polite offer ("Would you like me to take you...")
        # Pattern 3: Shall/Should patterns ("SHALL I ESCORT YOU", "SHOULD I TAKE YOU")
        # Pattern 4: Want patterns ("DO YOU WANT ME TO TAKE YOU")
        is_escort_request = (
            ("confirm" in response_lower and "take" in response_lower) or
            ("would you like" in response_lower and "take you" in response_lower) or
            ("shall i" in response_lower and ("escort" in response_lower or "take" in response_lower)) or
            ("should i" in response_lower and ("escort" in response_lower or "take" in response_lower)) or
            ("want me to" in response_lower and ("escort" in response_lower or "take" in response_lower)) or
            ("do you want" in response_lower and ("escort" in response_lower or "take" in response_lower))
        )
        
        if is_escort_request:
            # Try to extract location from response
            location_found = None
            for location_name, location_id in self.location_mapping.items():
                if location_name in response_lower:
                    location_found = location_id
                    break
            
            # If no location in response but we have a last mentioned location, use it
            if not location_found and self.last_mentioned_location:
                location_found = self.last_mentioned_location
            
            if location_found:
                self.pending_escort = location_found
                print(f"⏳ Escort pending confirmation: {location_found}")
                return True
        
        return False
    
    def _handle_escort_confirmation(self, user_input: str) -> bool:
        """Check if user is confirming or rejecting pending escort."""
        if not self.pending_escort:
            return False
        
        user_lower = user_input.lower()
        
        # Check for confirmation
        if any(word in user_lower for word in self.confirmation_words):
            if hasattr(self, 'ros_node'):
                self.ros_node.publish_location(self.pending_escort)
                print(f"🚶 Escorting confirmed: {self.pending_escort}")
            self.pending_escort = None
            return True
        
        # Check for rejection
        if any(word in user_lower for word in self.rejection_words):
            print(f"❌ Escort cancelled: {self.pending_escort}")
            self.pending_escort = None
            return True
        
        return False

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
            
            # Track location mentions in user queries
            transcript_lower = transcript.lower()
            for location_name, location_id in self.location_mapping.items():
                if location_name in transcript_lower:
                    self.last_mentioned_location = location_id
                    print(f"📍 Location mentioned: {location_id}")
                    break
            
            # Check if user is confirming/rejecting a pending escort
            # But ONLY accept yes/no as confirmation, not full questions
            user_lower = transcript.lower().strip()
            is_short_response = len(user_lower.split()) <= 3
            
            # If pending escort and short response, check confirmation
            if self.pending_escort and is_short_response:
                if self._handle_escort_confirmation(transcript):
                    # If confirming, escort
                    if any(word in transcript.lower() for word in self.confirmation_words):
                        await self._speak(f"Sure. Please follow me.")
                    # If rejecting, acknowledge
                    elif any(word in transcript.lower() for word in self.rejection_words):
                        await self._speak("Okay, no problem.")
                    return True
            
            # If user asks a new question while escort pending, clear stale state
            if self.pending_escort and not is_short_response:
                print(f"⚠️  Clearing stale escort state: {self.pending_escort} (new query received)")
                self.pending_escort = None

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
        full_response = ""  # Track complete response for ROS2
        async for chunk in self.agent.think_stream(user_text):
            buffer += chunk
            full_response += chunk
            
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
        
        # Wait for audio buffer to fully play out before mic starts again
        await asyncio.sleep(0.5)  # 500ms delay to prevent echo
        
        # Check if response is asking for escort confirmation
        if full_response:
            self._check_escort_request(full_response)

    async def _speak(self, text: str):
        if not self.tts._is_connected:
            await self.tts.connect()
        if not self.audio_playback._is_playing:
            self.audio_playback.start()

        audio = self.tts.synthesize_stream(text)
        await self.audio_playback.play_stream(audio)
        
        # Wait for audio buffer to fully play out
        await asyncio.sleep(0.5)  # 500ms delay to prevent echo

    async def run(self):
        print("\nROBOT VOICE PIPELINE STARTED")

        try:
            await self.tts.connect()
            self.audio_playback.start()
        except Exception as e:
            print(f"Pre-connect failed: {e}")

        try:
            while True:
                # Skip conversational pipeline when escorting
                if self.is_escorting:
                    await asyncio.sleep(0.5)
                    continue
                
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
