"""
Audio Capture Module for Robot Voice Pipeline

Captures audio from the microphone in real-time using PyAudio.
Provides an async iterator for streaming clean PCM audio chunks to STT.
Optimized for AssemblyAI / real-time command recognition.
"""

import asyncio
from typing import AsyncIterator, Optional

import pyaudio
import audioop


class AudioCapture:
    """
    Captures audio from the microphone and provides it as an async stream.

    Audio format:
    - Sample rate: 16kHz
    - Channels: 1 (mono)
    - Sample width: 16-bit PCM
    - Chunk size: 1024 frames (~64ms)
    """

    def __init__(
        self,
        sample_rate: int = 16000,
        channels: int = 1,
        chunk_size: int = 1024,  # ~64ms (ideal for streaming STT)
        rms_threshold: int = 300,  # Silence filter (tune if needed)
        max_queue_size: int = 5,   # Prevent latency buildup
    ):
        self.sample_rate = sample_rate
        self.channels = channels
        self.chunk_size = chunk_size
        self.rms_threshold = rms_threshold

        self.format = pyaudio.paInt16  # 16-bit PCM
        self.sample_width = 2  # bytes

        self._audio: Optional[pyaudio.PyAudio] = None
        self._stream: Optional[pyaudio.Stream] = None

        self._audio_queue: Optional[asyncio.Queue] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        self._is_recording = False

    def start(self):
        """Start capturing audio from the microphone."""
        if self._is_recording:
            return

        self._loop = asyncio.get_event_loop()
        self._audio_queue = asyncio.Queue(maxsize=5)

        self._audio = pyaudio.PyAudio()
        self._stream = self._audio.open(
            format=self.format,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size,
            stream_callback=self._audio_callback,
        )

        self._is_recording = True
        self._stream.start_stream()

        print(f"🎤 Microphone started (16kHz, mono, PCM16)")

    def stop(self):
        """Stop capturing audio."""
        if not self._is_recording:
            return

        self._is_recording = False

        if self._stream:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None

        if self._audio:
            self._audio.terminate()
            self._audio = None

        print("🎤 Microphone stopped")

    def _audio_callback(self, in_data, frame_count, time_info, status):
        if not self._is_recording or not self._audio_queue:
            return (None, pyaudio.paContinue)

        rms = audioop.rms(in_data, self.sample_width)
        if rms < self.rms_threshold:
            return (None, pyaudio.paContinue)

        def _safe_put():
            try:
                self._audio_queue.put_nowait(in_data)
            except asyncio.QueueFull:
                pass  # Drop audio silently

        self._loop.call_soon_threadsafe(_safe_put)

        return (None, pyaudio.paContinue)


    async def stream(
        self,
        duration_seconds: Optional[float] = None,
        stop_event: Optional[asyncio.Event] = None
    ) -> AsyncIterator[bytes]:
        """
        Async generator yielding audio chunks.

        Args:
            duration_seconds: Optional duration limit
            stop_event: Optional asyncio.Event to stop recording

        Yields:
            Raw PCM audio chunks (bytes)
        """
        if not self._audio_queue:
            raise RuntimeError("AudioCapture not started")

        start_time = self._loop.time()

        try:
            while self._is_recording:
                if stop_event and stop_event.is_set():
                    break

                if duration_seconds is not None:
                    if (self._loop.time() - start_time) >= duration_seconds:
                        break

                audio_chunk = await self._audio_queue.get()
                yield audio_chunk

        finally:
            # Drain queue
            while not self._audio_queue.empty():
                try:
                    self._audio_queue.get_nowait()
                except asyncio.QueueEmpty:
                    break

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
