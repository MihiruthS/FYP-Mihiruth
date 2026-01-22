"""
Speech-to-Text Module for Robot Voice Pipeline

Uses AssemblyAI's real-time streaming API to convert speech to text.
Simplified version without complex event system - returns final transcripts only.
"""

import asyncio
import json
import os
from typing import AsyncIterator, Optional
from urllib.parse import urlencode

import websockets
from websockets.client import WebSocketClientProtocol


class SpeechToText:
    """
    AssemblyAI Speech-to-Text client for real-time transcription.
    
    Connects to AssemblyAI's WebSocket API and streams audio for transcription.
    Returns final transcripts when speech turns are completed.
    """
    
    def __init__(
        self,
        api_key: Optional[str] = None,
        sample_rate: int = 16000,
        word_boost: Optional[list] = None,
    ):
        self.api_key = api_key or os.getenv("ASSEMBLYAI_API_KEY")
        if not self.api_key:
            raise ValueError("ASSEMBLYAI_API_KEY environment variable is required")
        
        self.sample_rate = sample_rate
        self._ws: Optional[WebSocketClientProtocol] = None
        self._is_connected = False
        
        # Word boost for common terms to improve accuracy
        self.word_boost = [
            "Quanta",
            "department",
            "office",
            "computer",
            "lab",
            "conference",
            "seminar",
            "directions",
            "room",
            "lecturer",
            "what",
            "electronic",
            "telecommunications",
            "ENTC",
        ]

    
    async def connect(self):
        """Establish WebSocket connection to AssemblyAI with enhanced parameters."""
        if self._is_connected:
            return
        
        params = {
            "sample_rate": self.sample_rate,
            "encoding": "pcm_s16le", 
            "format_turns": "true",  # Get formatted complete turns
            "punctuate": "true",
            "smart_format": "true",
            "word_boost": json.dumps(self.word_boost),  # Boost accuracy for these words
            "boost_param": "high",  # High boost level for better recognition
        }
        
        url = f"wss://streaming.assemblyai.com/v3/ws?{urlencode(params)}"
        
        self._ws = await websockets.connect(
            url,
            additional_headers={"Authorization": self.api_key}
        )
        self._is_connected = True
        print("Connected to AssemblyAI STT")
    
    async def send_audio(self, audio_chunk: bytes):
        """Send audio chunk to AssemblyAI for transcription."""
        if not self._is_connected or not self._ws:
            await self.connect()
        
        if audio_chunk:
            await self._ws.send(audio_chunk)
    
    async def send_termination(self):
        """Send termination message to signal end of audio stream."""
        if self._is_connected and self._ws:
            try:
                # Send empty message or close frame to signal end
                await self._ws.send(json.dumps({"terminate_session": True}))
            except Exception as e:
                print(f"Could not send termination: {e}")
    
    async def receive_transcript(self) -> Optional[str]:
        """
        Receive transcript from AssemblyAI.
        
        Returns:
            Final transcript text when a turn is completed, None otherwise
        """
        if not self._is_connected or not self._ws:
            return None
        
        try:
            raw_message = await self._ws.recv()
            message = json.loads(raw_message)
            message_type = message.get("type")
            
            # Debug: print message type
            if message_type not in ["Begin"]:
                print(f"STT message type: {message_type}")
            
            if message_type == "Turn":
                transcript = message.get("transcript", "")
                turn_is_formatted = message.get("turn_is_formatted", False)
                
                print(f"Turn received - formatted: {turn_is_formatted}, text: '{transcript}'")
                
                if turn_is_formatted and transcript:
                    return transcript
            
            elif message_type == "Termination":
                print("AssemblyAI session terminated")
                return None
            
            elif "error" in message:
                print(f"AssemblyAI error: {message['error']}")
                return None
                
        except json.JSONDecodeError as e:
            print(f"JSON decode error: {e}")
        except websockets.exceptions.ConnectionClosed as e:
            print(f"STT WebSocket connection closed: {e}")
            self._is_connected = False
        except Exception as e:
            print(f"Unexpected error in receive_transcript: {e}")
            import traceback
            traceback.print_exc()
        
        return None
    
    async def transcribe_stream(
        self,
        audio_stream: AsyncIterator[bytes]
    ) -> AsyncIterator[str]:
        """
        Transcribe an audio stream and yield final transcripts.
        
        Args:
            audio_stream: Async iterator of audio chunks (PCM 16-bit)
        
        Yields:
            Final transcript strings
        """
        await self.connect()
        
        # Background task to send audio
        audio_send_complete = asyncio.Event()
        
        async def send_audio_task():
            try:
                chunk_count = 0
                async for audio_chunk in audio_stream:
                    await self.send_audio(audio_chunk)
                    chunk_count += 1
                print(f"Sent {chunk_count} audio chunks to STT")
                # Signal end of audio to AssemblyAI
                await self.send_termination()
            except Exception as e:
                print(f"Error sending audio: {e}")
                import traceback
                traceback.print_exc()
            finally:
                audio_send_complete.set()
        
        # Start sending audio in background
        send_task = asyncio.create_task(send_audio_task())
        
        try:
            # Receive transcripts in main loop
            received_any = False
            timeout_counter = 0
            max_empty_receives = 30  # ~3 seconds after audio completes
            
            while self._is_connected:
                try:
                    # Set a timeout for receiving to avoid hanging forever
                    transcript = await asyncio.wait_for(
                        self.receive_transcript(), 
                        timeout=0.5
                    )
                    
                    if transcript:
                        received_any = True
                        timeout_counter = 0
                        yield transcript
                        # After receiving a transcript, we're done
                        break
                    
                except asyncio.TimeoutError:
                    # No message received in timeout period
                    pass
                
                # If audio sending is complete and we've waited enough, exit
                if audio_send_complete.is_set():
                    timeout_counter += 1
                    if timeout_counter >= max_empty_receives:
                        if not received_any:
                            print("No transcripts received from STT")
                        break
                    
        finally:
            # Cleanup
            if not send_task.done():
                send_task.cancel()
                try:
                    await send_task
                except asyncio.CancelledError:
                    pass
            
            await self.close()
    
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
                print("Disconnected from AssemblyAI STT")
    
    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()
