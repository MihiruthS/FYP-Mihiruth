# Emotion System - Quick Reference

## 🎭 Supported Emotions

1. **neutral** - Calm, informational
2. **joy** - Happy, positive  
3. **fear** - Worried, anxious
4. **disgust** - Disapproval
5. **sadness** - Sad, apologetic
6. **anger** - Frustrated, stern
7. **surprise** - Amazed, shocked

## 🚀 Key Feature: ZERO LATENCY

The emotion system runs **completely in parallel** with speech:
- Robot starts speaking immediately (0ms delay)
- Emotion classification happens in background
- Face updates during speech without interruption

## 📁 Files Modified/Created

### New Files
- `src/robot_pipeline/emotion_classifier.py` - Emotion classification engine
- `docs/EMOTION_SYSTEM.md` - Complete documentation
- `scripts/test_emotions.py` - Test script for emotions
- `scripts/show_emotion_guide.py` - Visual guide

### Modified Files
- `src/robot_pipeline/pipeline.py` - Added async emotion classification
- `src/robot_pipeline/mouth_controller.py` - Added emotion display methods
- `src/robot_pipeline/ros2_main.py` - Connected mouth_controller to pipeline

## 🔧 How It Works

```python
# In pipeline.py
async def _stream_response(self, user_text: str):
    # After 20 chars, start emotion classification (non-blocking)
    emotion_task = asyncio.create_task(
        self._classify_and_display_emotion(full_response)
    )
    
    # Continue with speech (doesn't wait for emotion)
    audio = self.tts.synthesize_stream(sentence)
    await self.audio_playback.play_stream(audio)
```

**Result:** Speech starts at 0ms, emotion updates ~200ms later (during speech)

## 🧪 Testing

```bash
# Test emotion classifier
python scripts/test_emotions.py

# View emotion guide
python scripts/show_emotion_guide.py

# Run full pipeline with emotions
python src/robot_pipeline/ros2_main.py
```

## 🎨 Customization

### Adjust Facial Expressions

Edit `mouth_controller.py`:
```python
def _display_joy(self):
    self.ros_node.publish_motor_pos(self.upper_lid_channel, 1400*4)
    self.ros_node.publish_motor_pos(self.lower_lid_channel, 1300*4)
```

### Change Classification Prompt

Edit `emotion_classifier.py`:
```python
self.system_prompt = """You are an emotion classifier..."""
```

### Disable Emotions

Comment out in `pipeline.py`:
```python
# emotion_task = asyncio.create_task(...)
```

## 📊 Performance

- **Classification Time:** ~200-500ms (parallel)
- **Speech Latency Added:** 0ms
- **Cost:** ~$0.0005 per response (GPT-3.5-turbo)
- **Fallback:** Rule-based classifier if API fails

## ⚙️ Requirements

- OpenAI API key (in `.env`):
  ```
  OPENAI_API_KEY=your_key_here
  ```

## 🔍 Debugging

Check logs for:
```
😊 Setting emotion: joy
🎭 Emotion: neutral (mouth_controller not available)
⚠️  Emotion classification error: ..., using neutral
```

If emotions don't display:
1. Verify `pipeline.mouth_controller` is set in `ros2_main.py`
2. Check ROS2 node is publishing to `/motor_pos`
3. Verify OpenAI API key is valid

## 📖 Full Documentation

See `docs/EMOTION_SYSTEM.md` for complete details.

## 📡 ROS2 Integration

The current emotion is published to the `/current_emotion` ROS2 topic:

```bash
# Subscribe to emotion updates
ros2 topic echo /current_emotion

# Example output:
# data: 'joy'
# data: 'neutral'
# data: 'surprise'
```

This allows other ROS2 nodes to:
- React to emotional state changes
- Log emotion history
- Trigger behaviors based on emotions
- Display emotions on external interfaces

## 🎯 Example Usage

```python
from robot_pipeline.emotion_classifier import EmotionClassifier

classifier = EmotionClassifier()

# Async classification
emotion = await classifier.classify("I'm so happy!")
# Returns: "joy"

# Simple rule-based (no API)
emotion = classifier.classify_simple("Thank you!")
# Returns: "joy"
```

```python
from robot_pipeline.mouth_controller import MouthController

mouth_controller = MouthController(ros_node=ros_node)
mouth_controller.set_emotion("joy")  # Display joy on face
```

## 🌟 Benefits

✅ No latency added to speech pipeline
✅ Accurate AI-based emotion detection  
✅ 7 distinct facial expressions
✅ Graceful fallback on errors
✅ Caching for efficiency
✅ Easy to customize
✅ Works with existing pipeline

## 🚫 No Changes Required

The emotion system is **automatically enabled** - no configuration needed!
Just run the robot as usual:
```bash
python src/robot_pipeline/ros2_main.py
```
