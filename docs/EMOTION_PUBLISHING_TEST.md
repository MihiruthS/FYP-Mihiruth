# Testing Emotion Publishing

## Quick Test Guide

### 1. Start the Robot Pipeline

In terminal 1:
```bash
cd /home/quanta/Desktop/head-new/FYP-Mihiruth
source venv/bin/activate
python src/robot_pipeline/ros2_main.py
```

### 2. Monitor Emotions (Option A - Simple)

In terminal 2:
```bash
ros2 topic echo /current_emotion
```

You'll see:
```
data: 'neutral'
---
data: 'joy'
---
data: 'surprise'
---
```

### 3. Monitor Emotions (Option B - With Statistics)

In terminal 2:
```bash
cd /home/quanta/Desktop/head-new/FYP-Mihiruth
python scripts/test_emotion_publishing.py
```

This will show:
```
🎭 EMOTION LISTENER - Monitoring /current_emotion

😐 Emotion: NEUTRAL    (count: 1)
😊 Emotion: JOY        (count: 1)
😲 Emotion: SURPRISE   (count: 1)
```

Press Ctrl+C to see statistics.

## Test Questions to Try

Ask the robot these questions and watch the emotions change:

### 😊 Joy
- "Thank you so much for your help!"
- "That's wonderful news!"
- "You're amazing!"

### 😢 Sadness
- "I lost my important document"
- "I'm disappointed with the results"
- "That's unfortunate"

### 😲 Surprise
- "Wow! That's incredible!"
- "I can't believe that happened!"
- "Really? That's amazing!"

### 😐 Neutral
- "Where is the computer lab?"
- "What time does the library close?"
- "Can you tell me about the department?"

### 😠 Anger
- "This is absolutely ridiculous!"
- "I'm very frustrated!"
- "This is unacceptable!"

### 😨 Fear
- "I'm not sure if this will work"
- "I'm worried about this"
- "What if something goes wrong?"

### 🤢 Disgust
- "This room smells terrible"
- "That's completely unacceptable behavior"
- "That's disgusting!"

## Verify Publishing

### Check if topic exists:
```bash
ros2 topic list | grep emotion
```

Should show: `/current_emotion`

### Check message type:
```bash
ros2 topic info /current_emotion
```

Should show:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: N
```

### Check publishing rate:
```bash
ros2 topic hz /current_emotion
```

Will show rate after a few messages are published.

## Integration with Other Nodes

Other ROS2 nodes can subscribe to `/current_emotion`:

### Python Example:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.sub = self.create_subscription(
            String, '/current_emotion', 
            self.emotion_callback, 10
        )
    
    def emotion_callback(self, msg):
        emotion = msg.data
        print(f"Robot is feeling: {emotion}")
        # React to emotion...
```

### C++ Example:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyNode : public rclcpp::Node {
public:
    MyNode() : Node("my_node") {
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/current_emotion", 10,
            std::bind(&MyNode::emotion_callback, this, std::placeholders::_1)
        );
    }

private:
    void emotion_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Robot is feeling: %s", msg->data.c_str());
        // React to emotion...
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
```

## Troubleshooting

### No messages appearing?
1. Check if robot pipeline is running
2. Verify ROS2 is sourced: `source /opt/ros/<distro>/setup.bash`
3. Check if topic exists: `ros2 topic list`
4. Ask the robot a question to trigger emotion classification

### Wrong emotions?
- The AI classifier learns from context
- Adjust the system prompt in `emotion_classifier.py`
- Use `classify_simple()` for rule-based classification

### Messages not updating?
- Emotions only update when robot generates responses
- Same emotion won't republish (no change = no message)
- Try asking different types of questions

## What You Should See

```
Terminal 1 (Robot Pipeline):
😊 Setting emotion: joy
🎭 Emotion: joy

Terminal 2 (ROS2 Topic):
data: 'joy'
---
```

Both should show the same emotion at approximately the same time!
