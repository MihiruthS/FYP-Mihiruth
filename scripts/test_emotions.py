#!/usr/bin/env python3
"""
Test script for emotion classification and display.

Tests the emotion classifier with various sample texts and displays
the detected emotions without requiring full pipeline.
"""

import asyncio
import sys
from pathlib import Path
from dotenv import load_dotenv

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from robot_pipeline.emotion_classifier import EmotionClassifier


async def test_emotion_classifier():
    """Test emotion classifier with various sample texts."""
    
    print("🎭 Testing Emotion Classifier\n")
    print("=" * 60)
    
    classifier = EmotionClassifier()
    
    # Test cases with expected emotions
    test_cases = [
        ("Hello! How can I help you today?", "neutral"),
        ("I'm so happy to see you! This is wonderful news!", "joy"),
        ("I'm really worried about this situation.", "fear"),
        ("That's absolutely disgusting and unacceptable.", "disgust"),
        ("I'm sorry to hear that. This is very unfortunate.", "sadness"),
        ("This is completely unacceptable! I'm very frustrated.", "anger"),
        ("Wow! I can't believe this happened! That's amazing!", "surprise"),
        ("The computer lab is located on the second floor.", "neutral"),
        ("Thank you so much! I really appreciate your help!", "joy"),
    ]
    
    print("\n📝 Testing emotion classification...\n")
    
    for text, expected in test_cases:
        print(f"Text: {text}")
        print(f"Expected: {expected}")
        
        # Test async classification
        emotion = await classifier.classify(text)
        print(f"Result: {emotion}")
        
        # Show if it matches
        match = "✅" if emotion == expected else "⚠️ "
        print(f"{match} Match: {emotion == expected}\n")
    
    print("=" * 60)
    print("\n✅ Emotion classifier test complete!")
    print("\nNote: The classifier uses AI, so results may vary slightly")
    print("from expected values. This is normal behavior.")


async def test_simple_classifier():
    """Test the simple rule-based classifier."""
    
    print("\n\n🎯 Testing Simple Rule-Based Classifier\n")
    print("=" * 60)
    
    classifier = EmotionClassifier()
    
    test_cases = [
        "Thank you so much!",
        "I'm sorry about that",
        "That's amazing!",
        "I'm worried about this",
        "This is terrible!",
    ]
    
    for text in test_cases:
        emotion = classifier.classify_simple(text)
        print(f"{text:40} → {emotion}")
    
    print("=" * 60)


async def main():
    """Run all tests."""
    load_dotenv()
    
    try:
        await test_emotion_classifier()
        await test_simple_classifier()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    asyncio.run(main())
