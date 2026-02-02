# Emotion Classification System

## Overview

The emotion classification system enables the robot to display facial expressions based on the emotional content of its speech. The system classifies text into 7 emotions:

- **neutral**: Calm, informational responses
- **joy**: Happy, positive, enthusiastic
- **fear**: Worried, anxious, uncertain
- **disgust**: Disapproval, unpleasant topics
- **sadness**: Sad, disappointed, apologetic
- **anger**: Frustrated, annoyed, stern
- **surprise**: Unexpected information, amazement

## Key Design: Zero Latency

**The emotion classification runs completely in parallel with speech synthesis and playback, adding ZERO latency to the pipeline.**

### How It Works

1. **Async Processing**: Emotion classification uses `asyncio.create_task()` to run in the background
2. **Early Classification**: Emotion is classified as soon as 20 characters are available
3. **Non-Blocking**: The robot starts speaking immediately while classification happens in parallel
4. **Face Updates**: The facial expression updates during speech without interrupting audio

```
User Query → AI Response Stream → [Speech Synthesis] → Audio Playback
                                   ↓
                            [Emotion Classification] → Face Update
                                   ↑
                            (runs in parallel)
```

## Architecture

### 1. EmotionClassifier (`emotion_classifier.py`)

The main classifier uses OpenAI's GPT-3.5-turbo for accurate emotion detection:

```python
from robot_pipeline.emotion_classifier import EmotionClassifier

classifier = EmotionClassifier()

# Async classification (recommended)
emotion = await classifier.classify("I'm so happy to see you!")
# Returns: "joy"

# Simple rule-based classification (fast, no API call)
emotion = classifier.classify_simple("Thank you!")
# Returns: "joy"
```

**Features:**
- **Async API calls**: Non-blocking classification
- **Caching**: Reduces redundant API calls for similar text
- **Timeout handling**: Falls back to neutral if API is slow
- **Simple fallback**: Rule-based classifier for offline use

### 2. MouthController Emotions (`mouth_controller.py`)

The mouth controller displays emotions through eyelid positions:

```python
from robot_pipeline.mouth_controller import MouthController

mouth_controller = MouthController(ros_node=ros_node)

# Display emotion
mouth_controller.set_emotion("joy")    # Smiling eyes
mouth_controller.set_emotion("surprise")  # Wide eyes
mouth_controller.set_emotion("sadness")   # Drooped eyes
```

**Emotion Mappings:**
- **neutral**: Normal open eyes
- **joy**: Slightly narrowed (smiling eyes)
- **fear**: Wide open eyes
- **disgust**: Narrowed with slight squint
- **sadness**: Drooped eyelids
- **anger**: Very narrowed and intense
- **surprise**: Very wide open eyes

### 3. Pipeline Integration (`pipeline.py`)

The pipeline automatically classifies and displays emotions:

```python
async def _stream_response(self, user_text: str):
    # ... AI response generation ...
    
    # Start emotion classification in background (NON-BLOCKING)
    if len(full_response) >= 20:
        emotion_task = asyncio.create_task(
            self._classify_and_display_emotion(full_response)
        )
    
    # Continue with speech synthesis (doesn't wait for emotion)
    audio = self.tts.synthesize_stream(sentence)
    await self.audio_playback.play_stream(audio)
```

**The key method:**

```python
async def _classify_and_display_emotion(self, text: str):
    """Runs asynchronously without blocking speech."""
    emotion = await self.emotion_classifier.classify(text)
    if self.mouth_controller:
        self.mouth_controller.set_emotion(emotion)
```

## Usage

### Running with Emotions

The emotion system is **automatically enabled** when you run the robot:

```bash
# Activate virtual environment
source venv/bin/activate

# Run with ROS2 (includes emotion display)
python src/robot_pipeline/ros2_main.py
```

### Monitoring Emotions via ROS2

The current emotion is published to `/current_emotion`:

```bash
# In another terminal, subscribe to emotion updates
ros2 topic echo /current_emotion

# You'll see:
# data: 'neutral'
# ---
# data: 'joy'
# ---
# data: 'surprise'
```

**Available ROS2 Topics:**
- `/current_emotion` (String): Current emotion being displayed
- `/motor_pos` (Int32MultiArray): Servo positions for face
- `/locations` (String): Navigation destinations
- `/arrived` (Bool): Arrival notifications

### Testing Emotions

Test the emotion classifier independently:

```bash
python scripts/test_emotions.py
```

This will classify sample texts and show detected emotions.

## Performance

- **Emotion Classification Time**: ~200-500ms (runs in parallel)
- **Speech Start Latency**: **0ms added** (starts immediately)
- **API Calls**: 1 per response (cached for similar text)
- **Fallback**: Simple rule-based classifier if API fails

## Configuration

### Adjusting Emotion Sensitivity

Edit `emotion_classifier.py` to adjust the classification prompt:

```python
self.system_prompt = """You are an emotion classifier...
Respond with ONLY the emotion name, nothing else."""
```

### Customizing Facial Expressions

Edit `mouth_controller.py` to adjust eyelid positions:

```python
def _display_joy(self):
    # Eyes: slightly narrowed (smiling eyes)
    self.ros_node.publish_motor_pos(self.upper_lid_channel, 1400*4)
    self.ros_node.publish_motor_pos(self.lower_lid_channel, 1300*4)
```

### Disabling Emotions

To disable emotion display, comment out the emotion task in `pipeline.py`:

```python
# emotion_task = asyncio.create_task(
#     self._classify_and_display_emotion(full_response)
# )
```

## API Requirements

The emotion classifier requires an OpenAI API key:

```bash
# In .env file
OPENAI_API_KEY=your_key_here
```

**Cost**: ~$0.0005 per classification (GPT-3.5-turbo)

## Troubleshooting

### Emotions Not Displaying

1. Check if `mouth_controller` is connected:
   ```python
   # In ros2_main.py, this line should exist:
   pipeline.mouth_controller = mouth_controller
   ```

2. Verify ROS2 node is running and publishing to `/motor_pos`

3. Check console for emotion logs:
   ```
   😊 Setting emotion: joy
   ```

### Classification Errors

If you see warnings like:
```
⚠️  Emotion classification error: ..., using neutral
```

**Solutions:**
- Check OpenAI API key is valid
- Verify internet connection
- Check API rate limits
- System will gracefully fall back to "neutral"

### Wrong Emotions

The classifier uses AI and may occasionally misclassify:
- Increase temperature in `emotion_classifier.py` for more varied results
- Decrease temperature for more consistent results
- Use `classify_simple()` for rule-based classification

## Future Enhancements

Possible improvements:

1. **Local Model**: Use a local transformer model (e.g., DistilBERT) to eliminate API dependency
2. **Emotion Transitions**: Smooth transitions between emotions
3. **Compound Emotions**: Detect multiple emotions in longer responses
4. **Context Awareness**: Use conversation history for better classification
5. **Custom Training**: Fine-tune model on robot-specific interactions

## Example Flow

```
User: "Where is the computer lab?"

1. AI generates: "The computer lab is on the second floor..."
   
2. Pipeline starts speaking immediately (no wait)
   
3. In parallel:
   - TTS converts to audio
   - Audio plays through speakers
   - Mouth syncs with audio
   - Emotion classifier analyzes text → "neutral"
   - Face updates to neutral expression
   
4. Total latency added: 0ms (all parallel)
```

## Technical Details

### Why Async?

```python
# ❌ BAD: Blocking (adds latency)
emotion = classify_sync(text)  # Waits ~500ms
speak(text)  # Speech delayed by 500ms

# ✅ GOOD: Non-blocking (zero latency)
asyncio.create_task(classify_and_display(text))  # Runs in background
speak(text)  # Starts immediately
```

### Threading vs Asyncio

We use `asyncio` instead of threads because:
- Better integration with the existing async pipeline
- Lower overhead than threads
- Easier to manage task lifecycle
- Natural fit for I/O-bound operations (API calls)

## Summary

The emotion classification system provides expressive facial feedback without impacting speech latency. It achieves this through:

1. **Parallel Processing**: Classification runs alongside speech
2. **Early Start**: Begins as soon as 20 chars are available
3. **Graceful Degradation**: Falls back to neutral on errors
4. **Minimal Overhead**: ~$0.0005 per response, 0ms latency

The robot can now express emotions naturally while maintaining its responsive, real-time interaction capabilities.
