#!/usr/bin/env python3
"""
Test script to verify ReSpeaker 4 Mic Array configuration and performance.
"""

import pyaudio
import audioop
import time
import sys


def list_audio_devices():
    """List all available audio input devices."""
    print("=" * 60)
    print("Available Audio Input Devices:")
    print("=" * 60)
    
    p = pyaudio.PyAudio()
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    
    respeaker_index = None
    
    for i in range(0, numdevices):
        device_info = p.get_device_info_by_host_api_device_index(0, i)
        max_input_channels = device_info.get('maxInputChannels', 0)
        
        if max_input_channels > 0:
            name = device_info.get('name')
            sample_rate = device_info.get('defaultSampleRate')
            
            marker = ""
            if 'respeaker' in name.lower():
                marker = " <-- ReSpeaker Detected"
                respeaker_index = i
            
            print(f"[{i}] {name}")
            print(f"    Max Input Channels: {max_input_channels}")
            print(f"    Default Sample Rate: {sample_rate} Hz{marker}")
            print()
    
    p.terminate()
    return respeaker_index


def test_microphone(device_index=None, duration=5):
    """Test microphone audio capture and measure performance."""
    print("=" * 60)
    print(f"Testing Microphone (device_index={device_index})")
    print("=" * 60)
    
    CHUNK = 3200  # ~200ms at 16kHz
    FORMAT = pyaudio.paInt16
    CHANNELS = 1
    RATE = 16000
    
    p = pyaudio.PyAudio()
    
    try:
        stream = p.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            input_device_index=device_index,
            frames_per_buffer=CHUNK
        )
        
        print(f"Recording for {duration} seconds...")
        print("Speak into the microphone...")
        print()
        
        start_time = time.time()
        frames = []
        rms_values = []
        
        while (time.time() - start_time) < duration:
            data = stream.read(CHUNK, exception_on_overflow=False)
            frames.append(data)
            
            # Calculate RMS (volume level)
            rms = audioop.rms(data, 2)  # 2 bytes per sample (16-bit)
            rms_values.append(rms)
            
            # Visual indicator
            bars = min(50, int(rms / 50))
            print(f"\rVolume: {'█' * bars:<50} RMS: {rms:>5}", end='', flush=True)
        
        print("\n")
        stream.stop_stream()
        stream.close()
        
        # Statistics
        avg_rms = sum(rms_values) / len(rms_values)
        max_rms = max(rms_values)
        min_rms = min(rms_values)
        
        print(f"Total frames captured: {len(frames)}")
        print(f"Average RMS: {avg_rms:.2f}")
        print(f"Max RMS: {max_rms}")
        print(f"Min RMS: {min_rms}")
        print()
        
        # Recommendations
        if avg_rms < 100:
            print("⚠️  WARNING: Audio level very low - speak louder or adjust microphone gain")
        elif avg_rms < 200:
            print("⚠️  Audio level somewhat low - consider speaking louder")
        elif avg_rms > 3000:
            print("⚠️  WARNING: Audio level very high - may cause distortion")
        else:
            print("✓ Audio levels look good!")
        
        print()
        print(f"Recommended RMS threshold: {int(avg_rms * 0.3)}")
        
    except Exception as e:
        print(f"\n❌ Error: {e}")
        return False
    finally:
        p.terminate()
    
    return True


def main():
    print("\n🎤 ReSpeaker 4 Mic Array Test Script\n")
    
    # List devices
    respeaker_index = list_audio_devices()
    
    if respeaker_index is None:
        print("⚠️  WARNING: ReSpeaker not detected!")
        print("Using default microphone instead...")
        print()
    
    # Test with ReSpeaker or default
    if not test_microphone(device_index=respeaker_index, duration=5):
        sys.exit(1)
    
    print("=" * 60)
    print("Test Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
