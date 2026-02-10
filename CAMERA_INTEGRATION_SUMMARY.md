# Camera User Detection Integration - Summary

## What Was Implemented

The robot can now identify users by name through camera feed integration. The system subscribes to the `/active_users` ROS2 topic and processes real-time user detection data.

## Changes Made

### 1. ROS2 Bridge Updates ([src/robot_pipeline/ros2_bridge.py](../src/robot_pipeline/ros2_bridge.py))

**Added:**
- Import for `custom_interfaces.msg` (PeopleArray, People)
- `active_users` dictionary to track current users
- `greeted_users` set to prevent duplicate greetings
- Subscriber for `/active_users` topic
- `active_users_callback()` method to process camera data
- `_greet_user()` method for automatic name-based greetings

**Behavior:**
- When user appears: Robot greets "Hello [name]! Nice to see you."
- Tracks all visible users with their IDs, names, and positions
- Updates pipeline with current user list
- Logs user entries and exits

### 2. Pipeline Updates ([src/robot_pipeline/pipeline.py](../src/robot_pipeline/pipeline.py))

**Added:**
- `active_users` list attribute
- `get_active_users_info()` method - formats user data for AI context
- `get_user_by_name()` method - retrieves user by name (case-insensitive)
- Passes active users info to AI agent during responses

### 3. AI Agent Updates ([src/robot_pipeline/ai/agent.py](../src/robot_pipeline/ai/agent.py))

**Modified:**
- `think()` method - accepts `active_users_info` parameter
- `think_stream()` method - accepts `active_users_info` parameter  
- `_build_system_prompt()` - includes user context in system prompt

### 4. Prompt Generator Updates ([src/robot_pipeline/ai/prompts.py](../src/robot_pipeline/ai/prompts.py))

**Modified:**
- `_base_prompt()` method - accepts `active_users_info` parameter
- Added "CAMERA INFORMATION" section to system prompt
- Robot now knows who it can see and can reference them

## How It Works

### Data Flow

```
Camera Node
    ↓ (publishes to /active_users)
ROS2 Bridge (VoicePipelineNode)
    ↓ (active_users_callback)
User Tracking & Greeting
    ↓ (updates pipeline.active_users)
Pipeline
    ↓ (get_active_users_info)
AI Agent
    ↓ (system prompt with user context)
Robot Response (aware of visible users)
```

### Example Usage

**Topic Data:**
```bash
ros2 topic echo /active_users --once
data:
- id: 5
  name: mihiruth
  hor_angle: -29
  ver_angle: -13
  missed_frames: 0
- id: 6
  name: nidula  
  hor_angle: 8
  ver_angle: -1
  missed_frames: 0
```

**Robot Behavior:**

1. **User Detection (Silent Tracking):**
   ```
   [mihiruth enters frame]
   [System logs: New user detected: mihiruth (ID: 5)]
   [No greeting - silent tracking]
   
   [nidula enters frame]
   [System logs: New user detected: nidula (ID: 6)]
   [No greeting - silent tracking]
   ```

2. **Wake Word with Personalized Greeting:**
   ```
   User: "Hey Quanta"
   Robot: "Hello Mihiruth, how can I help you today?"
   
   [If no users visible]
   User: "Hey Quanta"
   Robot: "Hello! How can I help you?"
   ```

3. **User Queries:**
   ```
   User: "Who can you see?"
   Robot: "I can currently see mihiruth and nidula."
   
   User: "Is anyone here?"
   Robot: "Yes, I can see mihiruth and nidula in front of me."
   
   User: "Do you know my name?"
   Robot: "Yes, your name is Mihiruth. How can I assist you today?"
   ```

4. **User Exits:**
   ```
   [mihiruth leaves frame]
   [System logs: User left: mihiruth (ID: 5)]
   [No announcement - silent tracking]
   ```

## Message Format

The system expects this message structure:

```
custom_interfaces/msg/PeopleArray

PeopleArray.msg:
  People[] data

People.msg:
  int16 id            # Unique identifier
  string name         # User's name  
  int16 hor_angle     # Horizontal viewing angle
  int16 ver_angle     # Vertical viewing angle
  int16 missed_frames # Tracking reliability metric
```

## Testing

### Quick Test
```bash
cd /home/quanta/Desktop/head-new/FYP-Mihiruth
python scripts/test_user_detection.py
```

### Full Integration Test
```bash
# Terminal 1: Start the robot system
cd /home/quanta/Desktop/head-new/FYP-Mihiruth
source venv/bin/activate
source /home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.zsh
python src/robot_pipeline/ros2_main.py

# Terminal 2: Check camera feed
source /home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.zsh
ros2 topic echo /active_users

# Walk into camera view - robot should greet you
# Ask "Who can you see?" - robot should respond with names
```

## Files Created

1. **[scripts/test_user_detection.py](../scripts/test_user_detection.py)**
   - Demonstration script showing expected behavior
   - Example data format
   - Integration checklist

2. **[docs/CAMERA_INTEGRATION.md](CAMERA_INTEGRATION.md)**
   - Complete documentation
   - API reference
   - Configuration options
   - Troubleshooting guide

## Next Steps

The integration is complete and ready to use. To activate:

1. Ensure your ROS2 workspace has `custom_interfaces` package built
2. Source the ROS2 workspace: `source /home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.zsh`
3. Verify camera node is publishing to `/active_users`
4. Start the robot system: `python src/robot_pipeline/ros2_main.py`
5. Users will be automatically detected and greeted

## Benefits

✅ **Personalized Interaction**: Robot addresses users by name  
✅ **Context Awareness**: AI knows who is present during conversations  
✅ **Automatic Greeting**: No manual trigger needed  
✅ **Multi-User Support**: Handles multiple people simultaneously  
✅ **Seamless Integration**: Works with existing voice pipeline  

---

**Status:** ✅ COMPLETE  
**Ready for:** Production testing with camera feed

