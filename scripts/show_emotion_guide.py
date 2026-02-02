"""
Visual demonstration of emotion display system.

Shows how different emotions look on the robot's face (via servo positions).
Useful for testing and calibrating facial expressions.
"""

import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))


def display_emotion_guide():
    """Display a guide showing all emotions and their servo positions."""
    
    print("\n" + "="*70)
    print("  ROBOT EMOTION DISPLAY GUIDE")
    print("="*70 + "\n")
    
    emotions = {
        "neutral": {
            "description": "Calm, informational, factual responses",
            "eyes": "Normal open position",
            "upper_lid": "1200*4",
            "lower_lid": "1120*4",
            "example": "The computer lab is on the second floor."
        },
        "joy": {
            "description": "Happy, positive, enthusiastic",
            "eyes": "Slightly narrowed (smiling eyes)",
            "upper_lid": "1400*4",
            "lower_lid": "1300*4",
            "example": "I'm so glad I could help you!"
        },
        "fear": {
            "description": "Worried, anxious, uncertain",
            "eyes": "Wide open",
            "upper_lid": "900*4",
            "lower_lid": "1000*4",
            "example": "I'm not sure about this situation."
        },
        "disgust": {
            "description": "Disapproval, unpleasant topics",
            "eyes": "Narrowed with squint",
            "upper_lid": "1500*4",
            "lower_lid": "1350*4",
            "example": "That's quite unpleasant."
        },
        "sadness": {
            "description": "Sad, disappointed, apologetic",
            "eyes": "Drooped eyelids",
            "upper_lid": "1600*4",
            "lower_lid": "1200*4",
            "example": "I'm sorry to hear that."
        },
        "anger": {
            "description": "Frustrated, annoyed, stern",
            "eyes": "Very narrowed and intense",
            "upper_lid": "1700*4",
            "lower_lid": "1400*4",
            "example": "This is completely unacceptable!"
        },
        "surprise": {
            "description": "Unexpected, amazed, shocked",
            "eyes": "Very wide open",
            "upper_lid": "800*4",
            "lower_lid": "900*4",
            "example": "Wow! That's incredible!"
        }
    }
    
    for emotion, details in emotions.items():
        print(f"┌─ {emotion.upper()}")
        print(f"│")
        print(f"│  Description: {details['description']}")
        print(f"│  Eyes:        {details['eyes']}")
        print(f"│  Upper Lid:   Channel 14 → {details['upper_lid']}")
        print(f"│  Lower Lid:   Channel 15 → {details['lower_lid']}")
        print(f"│")
        print(f"│  Example: \"{details['example']}\"")
        print(f"└" + "─"*68 + "\n")
    
    print("="*70)
    print("\n💡 CALIBRATION TIPS:")
    print("   - Adjust servo positions in mouth_controller.py")
    print("   - Higher upper_lid = eyes more closed")
    print("   - Lower lower_lid = eyes more open")
    print("   - Test with: mouth_controller.set_emotion('emotion_name')")
    print("\n" + "="*70 + "\n")


def display_pipeline_flow():
    """Display the emotion processing pipeline flow."""
    
    print("\n" + "="*70)
    print("  EMOTION PROCESSING PIPELINE")
    print("="*70 + "\n")
    
    flow = """
    ┌─────────────────────────────────────────────────────────────────┐
    │  USER QUERY                                                     │
    │  "Where is the computer lab?"                                   │
    └──────────────┬──────────────────────────────────────────────────┘
                   │
                   ▼
    ┌──────────────────────────────────────────────────────────────────┐
    │  AI AGENT (thinks & generates response)                          │
    │  "The computer lab is on the second floor..."                    │
    └──────────────┬───────────────────────────────────────────────────┘
                   │
                   │ (response text stream)
                   │
                   ▼
    ┌──────────────────────────────────────────────────────────────────┐
    │  PIPELINE: _stream_response()                                    │
    │  ┌─────────────────────┐    ┌──────────────────────────────┐    │
    │  │ After 20+ chars:    │    │ Main Path (no blocking):    │    │
    │  │                     │    │                              │    │
    │  │ asyncio.create_task │    │ • TTS Synthesis             │    │
    │  │   (Emotion          │    │ • Audio Playback            │    │
    │  │    Classification)  │    │ • Mouth Sync                │    │
    │  │                     │    │                              │    │
    │  │ RUNS IN PARALLEL ━━━━━━━━▶ NO LATENCY ADDED            │    │
    │  │ (background task)   │    │                              │    │
    │  └─────────┬───────────┘    └──────────────────────────────┘    │
    └────────────┼──────────────────────────────────────────────────────┘
                 │
                 ▼
    ┌──────────────────────────────────────────────────────────────────┐
    │  EMOTION CLASSIFIER (async)                                      │
    │  • Calls OpenAI API (~200-500ms)                                 │
    │  • Classifies: "neutral"                                         │
    └──────────────┬───────────────────────────────────────────────────┘
                   │
                   ▼
    ┌──────────────────────────────────────────────────────────────────┐
    │  MOUTH CONTROLLER                                                │
    │  • set_emotion("neutral")                                        │
    │  • Updates servo positions:                                      │
    │    - Upper lid: 1200*4                                           │
    │    - Lower lid: 1120*4                                           │
    └──────────────┬───────────────────────────────────────────────────┘
                   │
                   ▼
    ┌──────────────────────────────────────────────────────────────────┐
    │  ROS2 NODE                                                       │
    │  • Publishes to /motor_pos                                       │
    │  • Physical servos move                                          │
    │  • Face displays neutral expression                              │
    └──────────────────────────────────────────────────────────────────┘
    
    ⏱️  TOTAL LATENCY ADDED TO SPEECH: 0ms
    
    The robot starts speaking immediately while emotion classification
    happens in the background. The face updates during speech without
    interrupting audio playback.
    """
    
    print(flow)
    print("="*70 + "\n")


def display_timing_diagram():
    """Display timing diagram showing parallel execution."""
    
    print("\n" + "="*70)
    print("  TIMING DIAGRAM: Zero Latency Design")
    print("="*70 + "\n")
    
    timing = """
    Timeline (milliseconds) →
    
    0ms    500ms   1000ms  1500ms  2000ms  2500ms  3000ms
    │      │       │       │       │       │       │
    │
    ├─────────────────────────────────────────────────────────────►
    │ SPEECH SYNTHESIS & PLAYBACK (main path)
    │ ▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
    │ Audio starts at 0ms (NO DELAY)
    │
    │
    ├────┐ (20 chars available)
    │    │
    │    └──────────────────────────►
    │    EMOTION CLASSIFICATION (background task)
    │    ░░░░░░░░░░░░░░░░░░░░
    │    Classifies: ~200-500ms
    │    Updates face during speech
    │
    │
    │ ✅ Result: Robot speaks at 0ms, face updates at ~200ms
    │    (during ongoing speech, no interruption)
    
    
    COMPARISON:
    
    ❌ Blocking Approach (adds latency):
    │
    0ms────────────500ms───────────3000ms
    │              │               │
    ├──────────────┤               
    │ WAIT FOR     │               
    │ EMOTION      │               
    │ (500ms)      │               
    │              ├───────────────┤
    │              │ SPEECH        │
    │              │ (starts late) │
    │              │               │
    └──────────────┴───────────────┘
    
    Speech delayed by 500ms! ❌
    
    
    ✅ Async Approach (zero latency):
    │
    0ms────────────────────────────3000ms
    │                              │
    ├──────────────────────────────┤
    │ SPEECH (starts immediately)  │
    ├────┐                         │
    │    └──────────┐              
    │ EMOTION       │              
    │ (parallel)    │              
    └───────────────┴──────────────┘
    
    Speech starts at 0ms! ✅
    """
    
    print(timing)
    print("="*70 + "\n")


if __name__ == "__main__":
    display_emotion_guide()
    display_pipeline_flow()
    display_timing_diagram()
    
    print("\n💡 To test emotions:")
    print("   python scripts/test_emotions.py")
    print("\n💡 To run with full pipeline:")
    print("   python src/robot_pipeline/ros2_main.py")
    print()
