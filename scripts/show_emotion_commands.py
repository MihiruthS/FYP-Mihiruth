#!/usr/bin/env python3
"""
Test Emotion Display Commands

This script shows examples of commands you can use to ask the robot
to display emotions on command.
"""

def print_commands():
    """Display all emotion command examples."""
    
    print("\n" + "="*70)
    print("  🎭 EMOTION DISPLAY COMMANDS")
    print("="*70 + "\n")
    
    print("The robot can now display emotions on command!\n")
    
    print("📋 SHOW ALL EMOTIONS:")
    print("   • 'Can you show me your range of emotions?'")
    print("   • 'Show me all your emotions'")
    print("   • 'Display all your expressions'")
    print("   • 'Let me see all emotions'\n")
    
    print("😊 SHOW SPECIFIC EMOTION:")
    print("   • 'Can you show me a happy face?'")
    print("   • 'Show me anger'")
    print("   • 'Display the sadness emotion'")
    print("   • 'Let me see surprise'")
    print("   • 'Show your neutral expression'\n")
    
    print("🎯 EMOTION KEYWORDS:")
    emotions = {
        "😐 Neutral": ["neutral", "calm", "normal"],
        "😊 Joy": ["joy", "happy", "happiness", "smile", "cheerful"],
        "😨 Fear": ["fear", "scared", "afraid", "worried"],
        "🤢 Disgust": ["disgust", "disgusted", "gross"],
        "😢 Sadness": ["sad", "sadness", "unhappy"],
        "😠 Anger": ["anger", "angry", "mad", "furious"],
        "😲 Surprise": ["surprise", "surprised", "shocked", "amazed"]
    }
    
    for emotion, keywords in emotions.items():
        keyword_str = ", ".join(keywords)
        print(f"   {emotion:15} → {keyword_str}")
    
    print("\n" + "="*70)
    print("\n💡 EXAMPLE CONVERSATIONS:\n")
    
    examples = [
        ("You: Can you show me your range of emotions?",
         "Robot: Let me show you all my emotions.\n       [Shows: neutral, joy, surprise, sadness, anger, fear, disgust]"),
        
        ("You: Show me a happy face",
         "Robot: Here is my joy face.\n       [Displays happy expression]"),
        
        ("You: Can you show me anger?",
         "Robot: Here is my anger face.\n       [Displays angry expression]"),
        
        ("You: Display surprise and fear",
         "Robot: Let me show you those emotions.\n       [Shows surprise, then fear]"),
        
        ("You: Let me see your sad expression",
         "Robot: Here is my sadness face.\n       [Displays sad expression]"),
    ]
    
    for i, (user, robot) in enumerate(examples, 1):
        print(f"{i}. {user}")
        print(f"   {robot}\n")
    
    print("="*70)
    print("\n✨ FEATURES:\n")
    print("   ✓ Works alongside normal conversation")
    print("   ✓ Can show single or multiple emotions")
    print("   ✓ Cycles through all emotions on request")
    print("   ✓ Returns to neutral after display")
    print("   ✓ Publishes emotions to /current_emotion topic\n")
    
    print("="*70)
    print("\n🚀 TO USE:\n")
    print("   1. Start the robot: python src/robot_pipeline/ros2_main.py")
    print("   2. Wake it up: 'Hey Quanta'")
    print("   3. Ask to show emotions: 'Show me your emotions'\n")
    
    print("="*70 + "\n")


if __name__ == "__main__":
    print_commands()
