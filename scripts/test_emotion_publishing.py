#!/usr/bin/env python3
"""
Test script to verify emotion publishing to ROS2 topic.

This script demonstrates that emotions are being published to /current_emotion.
Run this alongside the main pipeline to see emotion updates in real-time.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


class EmotionListener(Node):
    """Simple ROS2 node that listens to emotion updates."""
    
    def __init__(self):
        super().__init__('emotion_listener')
        
        self.subscription = self.create_subscription(
            String,
            '/current_emotion',
            self.emotion_callback,
            10
        )
        
        self.emotion_count = {}
        
        print("\n" + "="*60)
        print("  🎭 EMOTION LISTENER - Monitoring /current_emotion")
        print("="*60)
        print("\nListening for emotion updates...")
        print("Press Ctrl+C to stop\n")
    
    def emotion_callback(self, msg):
        """Handle incoming emotion messages."""
        emotion = msg.data
        
        # Count emotions
        self.emotion_count[emotion] = self.emotion_count.get(emotion, 0) + 1
        
        # Display with emoji
        emoji_map = {
            "neutral": "😐",
            "joy": "😊",
            "fear": "😨",
            "disgust": "🤢",
            "sadness": "😢",
            "anger": "😠",
            "surprise": "😲"
        }
        
        emoji = emoji_map.get(emotion, "🎭")
        
        print(f"{emoji} Emotion: {emotion.upper():10} (count: {self.emotion_count[emotion]})")


def main():
    """Run the emotion listener."""
    
    try:
        rclpy.init()
        
        listener = EmotionListener()
        
        try:
            rclpy.spin(listener)
        except KeyboardInterrupt:
            print("\n\n" + "="*60)
            print("  EMOTION STATISTICS")
            print("="*60)
            
            if listener.emotion_count:
                total = sum(listener.emotion_count.values())
                print(f"\nTotal emotions detected: {total}\n")
                
                for emotion, count in sorted(listener.emotion_count.items(), 
                                            key=lambda x: x[1], reverse=True):
                    percentage = (count / total) * 100
                    print(f"  {emotion:10} : {count:3} times ({percentage:.1f}%)")
            else:
                print("\nNo emotions detected.")
            
            print("\n" + "="*60 + "\n")
        
        finally:
            listener.destroy_node()
            rclpy.shutdown()
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
        print("\nMake sure:")
        print("  1. ROS2 is installed and sourced")
        print("  2. The robot pipeline is running")
        print("  3. Emotions are being published to /current_emotion")
        sys.exit(1)


if __name__ == '__main__':
    main()
