"""
Robot Voice Pipeline - Main Orchestrator

A complete voice pipeline for robots: Listen → Think → Answer → Loop
"""

import asyncio
import os
import sys
from pathlib import Path
from dotenv import load_dotenv
import usb.core
import usb.util

from robot_pipeline.audio.capture import AudioCapture
from robot_pipeline.audio.playback import AudioPlayback
from robot_pipeline.speech.stt import SpeechToText
from robot_pipeline.speech.tts import TextToSpeech
from robot_pipeline.ai.agent import AIAgent
from robot_pipeline.ai.prompts import PromptGenerator
from robot_pipeline.ai.rag_database import RAGDatabase
from robot_pipeline.ai.faq_database import FAQDatabase
from robot_pipeline.emotion_classifier import EmotionClassifier

# Add usb_4_mic_array to path for DOA detection
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "usb_4_mic_array"))
try:
    from tuning import Tuning
    DOA_AVAILABLE = True
except ImportError:
    DOA_AVAILABLE = False
    print("⚠️  DOA (Direction of Arrival) detection not available - tuning module not found")


PROJECT_ROOT = Path(__file__).parent.parent.parent.absolute()
DOA_INTERACTION_MIN = -70
DOA_INTERACTION_MAX = 70


class RobotVoicePipeline:
    def __init__(self):
        self.is_awake = False
        self.is_escorting = False  # Track if robot is currently escorting
        self.pending_escort = None  # Tracks location waiting for confirmation
        self.pending_unsupported_escort = False  # Tracks unsupported escort confirmation prompts
        self.last_mentioned_location = None  # Track last location mentioned in conversation
        self.displaying_emotions = False  # Flag to prevent duplicate emotion classification during displays
        self.active_users = []  # List of currently visible users from camera
        self.current_user_name = None  # Currently active user name for chat history
        self.active_speaker = None  # Currently selected active speaker (People object)
        self.pending_name_learning = None  # ID of unknown user we're asking name for
        self.last_mapped_doa = None  # Last mapped DOA angle in robot frame (-77 to +77)
        
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
        
        # Emotion classifier for facial expressions
        self.emotion_classifier = EmotionClassifier()
        self.mouth_controller = None  # Will be set by ROS2 bridge
        
        # Direction of Arrival (DOA) detection
        self.doa_device = None
        self._init_doa()
        
        # Emotion keywords for command detection
        self.emotion_keywords = {
            "neutral": ["neutral", "calm", "normal"],
            "joy": ["joy", "happy", "happiness", "smile", "joyful", "cheerful"],
            "fear": ["fear", "scared", "afraid", "worried", "fearful"],
            "disgust": ["disgust", "disgusted", "gross"],
            "sadness": ["sad", "sadness", "unhappy", "disappointed"],
            "anger": ["anger", "angry", "mad", "furious"],
            "surprise": ["surprise", "surprised", "shocked", "amazed"]
        }

        print("Robot Voice Pipeline Initialized")

    def _init_doa(self):
        """Initialize Direction of Arrival (DOA) detection using ReSpeaker 4-mic array."""
        if not DOA_AVAILABLE:
            print("⚠️  DOA detection not available")
            return
        
        try:
            # Find ReSpeaker USB device (Vendor ID: 0x2886, Product ID: 0x0018)
            dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
            if dev:
                self.doa_device = Tuning(dev)
                print("✅ DOA (Direction of Arrival) detection initialized")
            else:
                print("⚠️  ReSpeaker 4-mic array not found - DOA unavailable")
        except Exception as e:
            print(f"⚠️  Failed to initialize DOA: {e}")
            self.doa_device = None
    
    def _print_doa(self):
        """Print the current Direction of Arrival (DOA) angle and publish to ROS2 if within range.
        
        Maps ReSpeaker DOA to robot reference frame:
        - < 103° → -77° (clamped to far left)
        - 103° → -77° (far left)
        - 180° → 0° (center)
        - 257° → +77° (far right)
        - > 257° → +77° (clamped to far right)
        """
        if self.doa_device:
            try:
                raw_doa = self.doa_device.direction
                print(f"🎯 Direction of Arrival (raw): {raw_doa}°")
                
                # Map DOA to robot reference frame with clamping
                if raw_doa < 103:
                    # Clamp to far left
                    mapped_doa = -77
                    print(f"   → Mapped DOA: {mapped_doa}° (clamped to far left)")
                elif raw_doa > 257:
                    # Clamp to far right
                    mapped_doa = 77
                    print(f"   → Mapped DOA: {mapped_doa}° (clamped to far right)")
                else:
                    # Within range: 180° → 0°, 103° → -77°, 257° → +77°
                    mapped_doa = raw_doa - 180
                    print(f"   → Mapped DOA: {mapped_doa}°")
                
                # Publish to ROS2 if available
                if hasattr(self, 'ros_node') and self.ros_node:
                    self.ros_node.publish_doa(mapped_doa)
                else:
                    print(f"   ✓ DOA mapped and ready for publishing (ROS2 not enabled)")

                # Cache most recent mapped DOA for active speaker selection
                self.last_mapped_doa = mapped_doa
            except Exception as e:
                print(f"⚠️  Failed to read DOA: {e}")
                self.last_mapped_doa = None
        # Silently skip if DOA not available

    def _is_doa_within_interaction_range(self) -> bool:
        """Return True when the latest mapped DOA is within the interaction range.

        If DOA is unavailable, allow interaction so the pipeline can still operate.
        """
        if self.last_mapped_doa is None:
            return True

        return DOA_INTERACTION_MIN <= self.last_mapped_doa <= DOA_INTERACTION_MAX

    def _clean_text(self, text: str) -> str:
        """Remove punctuation and normalize text for matching."""
        import string
        # Remove punctuation and convert to lowercase
        text = text.lower().strip()
        text = text.translate(str.maketrans('', '', string.punctuation))
        return text
    
    def _detect_emotion_display_request(self, text: str) -> dict:
        """
        Detect if user is asking to display an emotion.
        
        Returns:
            dict with 'is_request', 'emotions', 'show_all' keys
        """
        text_lower = text.lower()

        # Check for "all emotions" or "range of emotions"
        show_all = any(phrase in text_lower for phrase in [
            "all emotion", "range of emotion", "all expression",
            "different emotion", "every emotion", "each emotion"
        ])

        # Find which emotions are mentioned
        mentioned_emotions = []
        for emotion, keywords in self.emotion_keywords.items():
            if any(keyword in text_lower for keyword in keywords):
                mentioned_emotions.append(emotion)

        # Trigger phrases for showing/demonstrating something
        show_triggers = [
            "show me", "can you show", "display", "demonstrate",
            "let me see", "i want to see", "show your"
        ]
        has_show_trigger = any(trigger in text_lower for trigger in show_triggers)

        # Context words that indicate this is about facial emotions, not directions/instructions
        face_triggers = ["face", "expression", "emotion", "emotions"]
        has_face_context = any(trigger in text_lower for trigger in face_triggers)

        # Questions specifically asking how an emotion looks
        look_like_patterns = [
            "how does", "what does", "how do", "what do",
            "look like", "looks like"
        ]
        asks_look_like = any(pattern in text_lower for pattern in look_like_patterns)

        # Only treat as emotion-display request when emotion/face context is present.
        is_request = False
        if show_all:
            is_request = True
        elif has_show_trigger and (has_face_context or len(mentioned_emotions) > 0):
            is_request = True
        elif asks_look_like and len(mentioned_emotions) > 0:
            is_request = True

        if not is_request:
            return {"is_request": False, "emotions": [], "show_all": False}

        return {
            "is_request": True,
            "emotions": mentioned_emotions,
            "show_all": show_all
        }
    
    async def _display_emotions(self, emotions: list, show_all: bool = False):
        """
        Display requested emotions on the robot's face.
        
        Args:
            emotions: List of emotion names to display
            show_all: If True, cycle through all emotions
        """
        if not self.mouth_controller:
            await self._speak("I don't have access to my face controls right now.")
            return
        
        # Set flag to prevent duplicate emotion classification
        self.displaying_emotions = True
        
        if show_all:
            # Show all emotions in sequence
            all_emotions = ["neutral", "joy", "surprise", "sadness", "anger", "fear", "disgust"]
            
            # Natural phrases for each emotion
            emotion_phrases = {
                "neutral": "This is my neutral face",
                "joy": "This is me being happy",
                "surprise": "This is me surprised",
                "sadness": "This is a sad face",
                "anger": "This is me angry",
                "fear": "This is me scared",
                "disgust": "This is me showing disgust"
            }
            
            await self._speak("Let me show you all my emotions.")
            await asyncio.sleep(0.5)
            
            for emotion in all_emotions:
                print(f"Displaying: {emotion}")
                self.mouth_controller.set_emotion(emotion)
                phrase = emotion_phrases.get(emotion, f"This is {emotion}")
                await self._speak(phrase)
                await asyncio.sleep(1.8)  # Hold each emotion a bit longer before moving to next
                
            # Return to neutral
            self.mouth_controller.set_emotion("neutral")
            
            # Closing statement
            await self._speak("Those are the emotions that I have for now.")

            # Ensure final published/displayed emotion is neutral after closing speech.
            self.mouth_controller.set_emotion("neutral")
            
        elif emotions:
            # Show specific requested emotion(s)
            if len(emotions) == 1:
                emotion = emotions[0]
                self.mouth_controller.set_emotion(emotion)
                emotion_name = emotion.replace("_", " ")
                await self._speak(f"Here is my {emotion_name} face.")
            else:
                # Multiple emotions requested
                await self._speak("Let me show you those emotions.")
                await asyncio.sleep(0.5)
                
                for emotion in emotions:
                    emotion_name = emotion.replace("_", " ")
                    print(f"Displaying: {emotion}")
                    self.mouth_controller.set_emotion(emotion)
                    await self._speak(f"{emotion_name}")
                    await asyncio.sleep(1.0)
                
                # Return to neutral
                self.mouth_controller.set_emotion("neutral")
        else:
            # No specific emotion mentioned - show all
            await self._display_emotions([], show_all=True)
        
        # Clear flag after display is complete
        self.displaying_emotions = False
    
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
            ("do you want" in response_lower and ("escort" in response_lower or "take" in response_lower)) or
            ("escort" in response_lower and "confirm" in response_lower) or
            ("escort you" in response_lower and "there" in response_lower)
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
                self.pending_unsupported_escort = False
                print(f"⏳ Escort pending confirmation: {location_found}")
                return "pending"

            # Escort was requested, but destination is outside supported escort locations.
            self.pending_unsupported_escort = True
            print("⚠️  Escort request detected but no supported destination found")
            return "unsupported"
        
        self.pending_unsupported_escort = False
        return "none"
    
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
    
    def get_active_users_info(self) -> str:
        """Get formatted string of active users for context."""
        if not self.active_users:
            return "No users currently visible in camera."

        active_speaker_info = ""
        if self.active_speaker:
            active_speaker_info = (
                f"Active speaker for this utterance: {self.active_speaker.name} "
                f"(ID: {self.active_speaker.id}, horizontal: {self.active_speaker.hor_angle}°, "
                f"vertical: {self.active_speaker.ver_angle}°). "
                "Address this person by name when appropriate."
            )
        
        users_list = []
        for person in self.active_users:
            user_info = f"{person.name} (ID: {person.id})"
            if person.hor_angle or person.ver_angle:
                user_info += f" at angle (horizontal: {person.hor_angle}°, vertical: {person.ver_angle}°)"
            users_list.append(user_info)

        visible_users_info = "Currently visible users: " + ", ".join(users_list)
        if active_speaker_info:
            return active_speaker_info + "\n" + visible_users_info
        return visible_users_info
    
    def get_user_by_name(self, name: str):
        """Get user object by name (case-insensitive)."""
        name_lower = name.lower()
        for person in self.active_users:
            if person.name.lower() == name_lower:
                return person
        return None
    
    def _get_personalized_greeting(self) -> str:
        """Get greeting message, personalized if user is visible in camera."""
        if self.active_speaker:
            # Use detected active speaker for personalized greeting
            first_user = self.active_speaker
            
            # Check if user is unknown
            if first_user.name and first_user.name.lower() == 'unknown':
                self.pending_name_learning = first_user.id
                return "Hello! I don't think we've met before. May I know your name?"
            
            name = first_user.name.capitalize()
            return f"Hello {name}, how can I help you today?"
        else:
            # Default greeting if no users visible
            return "Hello! How can I help you?"

    def _select_active_speaker(self):
        """Select active speaker by closest horizontal angle to latest DOA."""
        if not self.active_users:
            return None

        # Fallback when DOA is not available yet
        if self.last_mapped_doa is None:
            return self.active_users[0]

        # Choose the person whose horizontal angle is closest to mapped DOA
        return min(
            self.active_users,
            key=lambda person: abs(person.hor_angle - self.last_mapped_doa)
        )
    
    def _extract_name_from_response(self, text: str) -> str:
        """Extract a person's name from their response."""
        # Common patterns: "My name is X", "I'm X", "I am X", "Call me X", or just "X"
        import re
        
        text_lower = text.lower()
        
        # Pattern 1: "my name is [name]"
        match = re.search(r"my name is ([a-z]+)", text_lower)
        if match:
            return match.group(1)
        
        # Pattern 2: "i'm [name]" or "i am [name]"
        match = re.search(r"i['']?m ([a-z]+)", text_lower)
        if match:
            return match.group(1)
        
        match = re.search(r"i am ([a-z]+)", text_lower)
        if match:
            return match.group(1)
        
        # Pattern 3: "call me [name]"
        match = re.search(r"call me ([a-z]+)", text_lower)
        if match:
            return match.group(1)
        
        # Pattern 4: Single word (likely just the name)
        words = text_lower.split()
        if len(words) == 1 and words[0].isalpha():
            return words[0]
        
        # Pattern 5: "it's [name]" or "this is [name]"
        match = re.search(r"it['']?s ([a-z]+)", text_lower)
        if match:
            return match.group(1)
        
        match = re.search(r"this is ([a-z]+)", text_lower)
        if match:
            return match.group(1)
        
        return None
    
    def _update_active_user(self):
        """Update active user based on camera data and switch conversation history."""
        if self.active_users and len(self.active_users) > 0:
            # Select speaker by DOA/hor_angle matching
            selected_user = self._select_active_speaker()
            self.active_speaker = selected_user

            # Publish exact selected People object to /active_speaker
            if selected_user and hasattr(self, 'ros_node') and self.ros_node:
                self.ros_node.publish_active_speaker(selected_user)

            new_user_name = selected_user.name.lower() if selected_user and selected_user.name else None
            
            if new_user_name and new_user_name != self.current_user_name:
                self.current_user_name = new_user_name
                self.agent.set_active_user(new_user_name)
                print(
                    f"Active user changed to: {selected_user.name} (ID: {selected_user.id}, "
                    f"hor_angle: {selected_user.hor_angle}°, doa: {self.last_mapped_doa}°)"
                )
        else:
            # No users visible - use generic conversation
            self.active_speaker = None
            if self.current_user_name is not None:
                self.current_user_name = None
                self.agent.set_active_user(None)
                print("No users visible - using generic conversation history")

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
                
                # Print direction of arrival
                self._print_doa()

            self.audio_capture.stop()

            if not transcript:
                return True

            # Clean text by removing punctuation for better matching
            text = self._clean_text(transcript)

            if any(w in text for w in self.wake_words):
                if not self._is_doa_within_interaction_range():
                    print(
                        f"🚫 Ignoring wake word outside DOA range: {self.last_mapped_doa}° "
                        f"(allowed: {DOA_INTERACTION_MIN}° to {DOA_INTERACTION_MAX}°)"
                    )
                    return True

                self.is_awake = True
                # Update active user before greeting
                self._update_active_user()
                # Greet with user's name if available from camera
                greeting = self._get_personalized_greeting()
                await self._speak(greeting)
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
                
                # Print direction of arrival
                self._print_doa()

            self.audio_capture.stop()

            if not transcript:
                return True

            if not self._is_doa_within_interaction_range():
                print(
                    f"🚫 Ignoring query outside DOA range: {self.last_mapped_doa}° "
                    f"(allowed: {DOA_INTERACTION_MIN}° to {DOA_INTERACTION_MAX}°)"
                )
                return True
            
            # Update active user based on who is currently visible
            self._update_active_user()
            
            # Check if we're learning a user's name
            if self.pending_name_learning is not None:
                learned_name = self._extract_name_from_response(transcript)
                if learned_name:
                    # Found a name! Publish it to /new_user
                    if hasattr(self, 'ros_node'):
                        self.ros_node.publish_new_user(self.pending_name_learning, learned_name)
                        print(f"✅ Learned name: {learned_name} (ID: {self.pending_name_learning})")
                        
                        # Thank the user
                        await self._speak(f"Nice to meet you, {learned_name.capitalize()}! How can I help you today?")
                        self.pending_name_learning = None
                        return True
                    else:
                        print("⚠️ Cannot publish new user - ROS node not available")
                        self.pending_name_learning = None
            
            # Track location mentions in the current query.
            # Clear stale location when query doesn't include an escortable destination.
            transcript_lower = transcript.lower()
            location_found_in_query = None
            for location_name, location_id in self.location_mapping.items():
                if location_name in transcript_lower:
                    location_found_in_query = location_id
                    print(f"📍 Location mentioned: {location_id}")
                    break

            self.last_mentioned_location = location_found_in_query
            
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

            # Handle confirmations for unsupported escort locations.
            if self.pending_unsupported_escort and is_short_response:
                if any(word in transcript.lower() for word in self.confirmation_words):
                    await self._speak(
                        "Sorry, I cannot escort to that location. "
                        "I can escort only to the Department Office, Computer Lab, "
                        "Head of the Department's Office, and Conference Room."
                    )
                    self.pending_unsupported_escort = False
                    return True
                if any(word in transcript.lower() for word in self.rejection_words):
                    await self._speak("Okay, no problem.")
                    self.pending_unsupported_escort = False
                    return True
            
            # If user asks a new question while escort pending, clear stale state
            if self.pending_escort and not is_short_response:
                print(f"⚠️  Clearing stale escort state: {self.pending_escort} (new query received)")
                self.pending_escort = None
            if self.pending_unsupported_escort and not is_short_response:
                self.pending_unsupported_escort = False
            
            # Check for emotion display requests
            emotion_request = self._detect_emotion_display_request(transcript)
            if emotion_request["is_request"]:
                print(f"🎭 Emotion display request detected")
                await self._display_emotions(
                    emotion_request["emotions"], 
                    emotion_request["show_all"]
                )
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

            # Ask unknown user's name after answering their question.
            if (self.active_users and len(self.active_users) > 0 and
                self.pending_name_learning is None):
                detected_user = self.active_speaker if self.active_speaker else self._select_active_speaker()
                if detected_user and detected_user.name and detected_user.name.lower() == 'unknown':
                    self.pending_name_learning = detected_user.id
                    await self._speak("By the way, I don't think we've met before. May I know your name?")

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
        
        # Start emotion classification in background (non-blocking)
        emotion_task = None
        
        # Get active users info for context
        active_users_info = self.get_active_users_info()
        
        async for chunk in self.agent.think_stream(user_text, active_users_info=active_users_info):
            buffer += chunk
            full_response += chunk
            
            # Start emotion classification early with first ~20 chars (runs in parallel)
            if emotion_task is None and len(full_response) >= 20:
                emotion_task = asyncio.create_task(
                    self._classify_and_display_emotion(full_response)
                )
            
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
        
        # Ensure emotion task completes (but don't block if it's not done yet)
        if emotion_task and not emotion_task.done():
            # Let it finish in background, don't wait
            pass
        
        # Check if response is asking for escort confirmation
        if full_response:
            escort_status = self._check_escort_request(full_response)
            if escort_status == "unsupported":
                await self._speak(
                    "I can give directions there, but I can escort only to the Department Office, "
                    "Computer Lab, Head of the Department's Office, and Conference Room."
                )
    
    async def _classify_and_display_emotion(self, text: str):
        """
        Classify emotion from text and display it on the robot's face.
        
        This runs asynchronously in the background without blocking speech.
        
        Args:
            text: The text to classify
        """
        try:
            # Classify emotion asynchronously
            emotion = await self.emotion_classifier.classify(text)
            
            # Display emotion on robot face
            if self.mouth_controller:
                self.mouth_controller.set_emotion(emotion)
            else:
                print(f"🎭 Emotion: {emotion} (mouth_controller not available)")
                
        except Exception as e:
            print(f"⚠️  Error in emotion classification: {e}")

    async def _speak(self, text: str):
        if not self.tts._is_connected:
            await self.tts.connect()
        if not self.audio_playback._is_playing:
            self.audio_playback.start()
        
        # Classify and display emotion in background (non-blocking)
        # Skip if we're already displaying emotions to avoid duplicates
        if not self.displaying_emotions:
            asyncio.create_task(self._classify_and_display_emotion(text))

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
