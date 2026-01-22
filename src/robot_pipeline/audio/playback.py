"""
Audio Playback Module for Robot Voice Pipeline

Plays audio through the system speakers using PyAudio.
Handles PCM audio format from TTS services.
"""

import asyncio
import queue
from typing import Optional

import pyaudio


class AudioPlayback:
    """
    Plays audio through the system speakers.
    
    Handles PCM audio format with configurable sample rate.
    Designed to work with Cartesia TTS output (24kHz, 16-bit PCM).
    """
    
    def __init__(
        self,
        sample_rate: int = 24000,
        channels: int = 1,
        chunk_size: int = 512,  # Smaller chunks for lower latency
    ):
        """
        Initialize audio playback.
        
        Args:
            sample_rate: Audio sample rate in Hz (default: 24000 for Cartesia)
            channels: Number of audio channels (default: 1 for mono)
            chunk_size: Size of audio chunks to play at once
        """
        self.sample_rate = sample_rate
        self.channels = channels
        self.chunk_size = chunk_size
        self.format = pyaudio.paInt16  # 16-bit PCM
        
        self._audio: Optional[pyaudio.PyAudio] = None
        self._stream: Optional[pyaudio.Stream] = None
        self._is_playing = False
    
    def start(self):
        """Initialize the audio playback system."""
        if self._is_playing:
            return
        
        self._audio = pyaudio.PyAudio()
        self._stream = self._audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            output=True,
            frames_per_buffer=self.chunk_size,
        )
        
        self._is_playing = True
        print(f"Audio playback started (sample_rate={self.sample_rate}Hz)")
    
    def stop(self):
        """Stop and cleanup audio playback."""
        if not self._is_playing:
            return
        
        self._is_playing = False
        
        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None
        
        if self._audio:
            self._audio.terminate()
            self._audio = None
        
        print("Audio playback stopped")
    
    def play(self, audio_data: bytes):
        """
        Play audio data synchronously.
        
        Args:
            audio_data: PCM audio bytes to play
        """
        if not self._is_playing:
            self.start()
        
        if not audio_data:
            return
        
        # Play audio in chunks
        for i in range(0, len(audio_data), self.chunk_size):
            chunk = audio_data[i:i + self.chunk_size]
            self._stream.write(chunk)
    
    async def play_async(self, audio_data: bytes):
        """
        Play audio data asynchronously.
        
        Args:
            audio_data: PCM audio bytes to play
        """
        if not self._is_playing:
            self.start()
        
        if not audio_data:
            return
        
        # Play audio in chunks with async yields
        for i in range(0, len(audio_data), self.chunk_size):
            chunk = audio_data[i:i + self.chunk_size]
            self._stream.write(chunk)
            # Yield control to allow other tasks to run
            await asyncio.sleep(0)
    
    async def play_stream(self, audio_stream):
        """
        Play audio from an async stream.
        
        Args:
            audio_stream: Async iterator yielding audio chunks
        """
        if not self._is_playing:
            self.start()
        
        async for audio_chunk in audio_stream:
            if audio_chunk:
                self._stream.write(audio_chunk)
                # Yield control to allow other tasks to run
                await asyncio.sleep(0)
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
    
    async def __aenter__(self):
        """Async context manager entry."""
        self.start()
        return self
    
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        self.stop()
