# Emotion Display Commands - Quick Reference

## 🗣️ What You Can Say

### Show All Emotions
```
"Can you show me your range of emotions?"
"Show me all your emotions"
"Display all your expressions"
"Let me see all emotions"
```
**Result:** Robot cycles through all 7 emotions (neutral, joy, surprise, sadness, anger, fear, disgust)

---

### Show Specific Emotion

#### Happy/Joy
```
"Show me a happy face"
"Can you show me joy?"
"Display happiness"
"Let me see a smile"
```

#### Sad
```
"Show me sadness"
"Can you show me a sad face?"
"Display sadness"
```

#### Angry
```
"Show me anger"
"Can you show me an angry face?"
"Display anger"
```

#### Surprised
```
"Show me surprise"
"Can you show me a surprised face?"
"Display surprise"
```

#### Scared/Fear
```
"Show me fear"
"Can you show me a scared face?"
"Display fear"
```

#### Disgusted
```
"Show me disgust"
"Display disgust"
```

#### Neutral/Calm
```
"Show me a neutral face"
"Display calm"
"Show me normal"
```

---

## 🎯 How It Works

1. **You ask:** "Show me anger"
2. **Robot detects:** Emotion display request
3. **Robot displays:** Sets face to angry expression
4. **Robot says:** "Here is my anger face"
5. **ROS2 publishes:** Emotion to `/current_emotion`

---

## 📝 Complete Example Conversation

```
You: Hey Quanta
Robot: Hello! How can I help you?

You: Can you show me your range of emotions?
Robot: Let me show you all my emotions.
       [Shows each emotion with name]
       "neutral"
       "joy"
       "surprise"
       "sadness"
       "anger"
       "fear"
       "disgust"

You: That was cool! Now show me just the happy face
Robot: Here is my joy face.
       [Displays happy expression]

You: Thanks! Where is the computer lab?
Robot: [Returns to normal conversation mode]
```

---

## ✨ Features

✅ **Natural language detection** - Understands various ways of asking
✅ **Multiple emotions** - Can show one or all emotions
✅ **Smooth transitions** - 1 second hold time per emotion
✅ **Voice feedback** - Announces each emotion being displayed
✅ **ROS2 integration** - Publishes to `/current_emotion` topic
✅ **Non-blocking** - Doesn't interfere with normal conversation

---

## 🔍 Detection Keywords

The robot recognizes these trigger phrases:
- "show me"
- "can you show"
- "display"
- "demonstrate"
- "let me see"
- "show your"

Combined with emotion keywords:
- Joy: happy, smile, cheerful, joyful
- Sadness: sad, unhappy, disappointed
- Anger: angry, mad, furious
- Surprise: surprised, shocked, amazed
- Fear: scared, afraid, worried, fearful
- Disgust: disgusted, gross
- Neutral: calm, normal

---

## 🎬 Try It Now!

1. **Start robot:**
   ```bash
   python src/robot_pipeline/ros2_main.py
   ```

2. **Wake up:**
   ```
   "Hey Quanta"
   ```

3. **Ask to show emotions:**
   ```
   "Can you show me your range of emotions?"
   ```

4. **Watch the magic!** 🎭
