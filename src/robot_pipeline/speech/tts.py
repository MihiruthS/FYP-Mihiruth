"""
Text-to-Speech Module for Robot Voice Pipeline

Uses Cartesia's Sonic API to convert text to natural-sounding speech.
Simplified version that returns complete audio for each text input.
"""

import asyncio
import base64
import json
import os
import time
from typing import AsyncIterator, Optional

import websockets
from websockets.client import WebSocketClientProtocol


class TextToSpeech:
    """
    Cartesia Text-to-Speech client for converting text to speech.
    
    Connects to Cartesia's WebSocket API and streams audio synthesis.
    Returns audio chunks that can be played back immediately.
    """
    
    def __init__(
        self,
        api_key: Optional[str] = None,
        voice_id: str = "a33f7a4c-100f-41cf-a1fd-5822e8fc253f",  
        model_id: str = "sonic-english",
        sample_rate: int = 24000,
        encoding: str = "pcm_s16le",
        language: str = "en",
        speed: str = "slowest",  # Options: "slowest", "slow", "normal", "fast", "fastest"
    ):
        """
        Initialize the TTS client.
        
        Args:
            api_key: Cartesia API key (reads from CARTESIA_API_KEY env var if not provided)
            voice_id: Voice ID to use (default: British Lady)
            model_id: Model ID (default: sonic-english)
            sample_rate: Audio sample rate in Hz (default: 24000)
            encoding: Audio encoding format (default: pcm_s16le)
            language: Language code (default: en)
            speed: Speech speed - "slowest", "slow", "normal", "fast", "fastest" (default: slow)
        """
        self.api_key = api_key or os.getenv("CARTESIA_API_KEY")
        if not self.api_key:
            raise ValueError("CARTESIA_API_KEY environment variable is required")
        
        self.voice_id = voice_id
        self.model_id = model_id
        self.sample_rate = sample_rate
        self.encoding = encoding
        self.language = language
        self.speed = speed  # Store speed as string: "slowest", "slow", "normal", "fast", "fastest"
        
        self._ws: Optional[WebSocketClientProtocol] = None
        self._is_connected = False
        self._context_counter = 0
    
    def _generate_context_id(self) -> str:
        """Generate a unique context ID for each TTS request."""
        timestamp = int(time.time() * 1000)
        counter = self._context_counter
        self._context_counter += 1
        return f"ctx_{timestamp}_{counter}"
    
    async def connect(self):
        """Establish WebSocket connection to Cartesia."""
        if self._is_connected:
            return
        
        url = (
            f"wss://api.cartesia.ai/tts/websocket"
            f"?api_key={self.api_key}&cartesia_version=2024-06-10"
        )
        
        self._ws = await websockets.connect(url)
        self._is_connected = True
        print("Connected to Cartesia TTS")
    
    async def synthesize(self, text: str) -> bytes:
        """
        Synthesize speech from text and return complete audio.
        
        Args:
            text: Text to convert to speech
        
        Returns:
            Complete audio as bytes (PCM format)
        """
        if not text.strip():
            return b""
        
        if not self._is_connected or not self._ws:
            await self.connect()
        
        print(f"Synthesizing: '{text[:50]}...'")
        
        # Send synthesis request - try speed at voice level
        voice_config = {
            "mode": "id",
            "id": self.voice_id,
        }
        
        # Add speed control if not normal
        if self.speed != "normal":
            voice_config["speed"] = self.speed
        
        payload = {
            "model_id": self.model_id,
            "transcript": text,
            "voice": voice_config,
            "output_format": {
                "container": "raw",
                "encoding": self.encoding,
                "sample_rate": self.sample_rate,
            },
            "language": self.language,
            "context_id": self._generate_context_id(),
        }
        
        await self._ws.send(json.dumps(payload))
        
        # Collect audio chunks
        audio_data = b""
        
        try:
            async for raw_message in self._ws:
                try:
                    message = json.loads(raw_message)
                    
                    # Check for audio data
                    if "data" in message and message["data"] is not None:
                        audio_chunk = base64.b64decode(message["data"])
                        if audio_chunk:
                            audio_data += audio_chunk
                    
                    # Check if synthesis is complete
                    if message.get("done"):
                        break
                    
                    # Check for errors
                    if "error" in message and message["error"]:
                        print(f"Cartesia error: {message['error']}")
                        break
                        
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                    continue
                    
        except websockets.exceptions.ConnectionClosed:
            print("TTS WebSocket connection closed")
            self._is_connected = False
            return b""
        
        print(f"Synthesized {len(audio_data)} bytes of audio")
        return audio_data
    
    async def synthesize_stream(self, text: str) -> AsyncIterator[bytes]:
        """
        Synthesize speech from text and stream audio chunks.
        
        Args:
            text: Text to convert to speech
        
        Yields:
            Audio chunks as they are generated
        """
        if not text.strip():
            return
        
        if not self._is_connected or not self._ws:
            await self.connect()
        
        print(f"Synthesizing (streaming): '{text[:50]}...'")
        
        # Send synthesis request
        # Try multiple approaches for speed control
        voice_config = {
            "mode": "id",
            "id": self.voice_id,
        }
        
        # Approach 1: Try speed at voice level (newer Cartesia API)
        if self.speed != "normal":
            voice_config["speed"] = self.speed
        
        payload = {
            "model_id": self.model_id,
            "transcript": text,
            "voice": voice_config,
            "output_format": {
                "container": "raw",
                "encoding": self.encoding,
                "sample_rate": self.sample_rate,
            },
            "language": self.language,
            "context_id": self._generate_context_id(),
        }
        
        await self._ws.send(json.dumps(payload))
        
        # Stream audio chunks
        try:
            async for raw_message in self._ws:
                try:
                    message = json.loads(raw_message)
                    
                    # Yield audio data
                    if "data" in message and message["data"] is not None:
                        audio_chunk = base64.b64decode(message["data"])
                        if audio_chunk:
                            yield audio_chunk
                    
                    # Check if synthesis is complete
                    if message.get("done"):
                        break
                    
                    # Check for errors
                    if "error" in message and message["error"]:
                        print(f"Cartesia error: {message['error']}")
                        break
                        
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                    continue
                    
        except websockets.exceptions.ConnectionClosed:
            print("TTS WebSocket connection closed")
            self._is_connected = False
    
    async def close(self):
        """Close the WebSocket connection."""
        if self._ws and self._is_connected:
            try:
                await self._ws.close()
            except:
                pass
            finally:
                self._ws = None
                self._is_connected = False
                print("Disconnected from Cartesia TTS")
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()
