# Camera Integration - Active User Detection

## Overview

The robot now integrates with a camera feed to detect and identify users in real-time. When users appear in the camera frame, the robot automatically greets them by name and can reference them throughout conversations.

## Features

### 1. **Silent User Tracking**
When a user appears in the camera frame, the robot silently tracks them without any announcement. This prevents interrupting ongoing conversations or activities.

### 2. **Personalized Wake Word Greeting**
When the wake word "Hey Quanta" is detected, the robot greets the user by name:
- "Hello [Name], how can I help you today?"
- Falls back to generic greeting if no users are visible

### 3. **User Awareness in Conversations**
The robot knows who is currently visible and can:
- Answer questions like "Who can you see?" or "Who is here?"
- Address users by name during conversations
- Provide context-aware responses based on who is present

### 4. **User Tracking**
- Tracks all users currently visible in frame
- Maintains user IDs, names, and positions
- Automatically updates when users enter or leave the frame
- Logs all user entry/exit events

## ROS2 Integration

### Topic: `/active_users`

**Message Type:** `custom_interfaces/msg/PeopleArray`

**Structure:**
```
PeopleArray:
  People[] data

People:
  int16 id            # Unique user ID
  string name         # User's name
  int16 hor_angle     # Horizontal angle (degrees)
  int16 ver_angle     # Vertical angle (degrees)
  int16 missed_frames # Number of consecutive missed frames
```

**Example Data:**
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

## Implementation Details

### Files Modified

1. **[src/robot_pipeline/ros2_bridge.py](../src/robot_pipeline/ros2_bridge.py)**
   - Added subscriber for `/active_users` topic
   - Implemented `active_users_callback()` to process camera data
   - Added `_greet_user()` method for automatic greetings
   - Tracks greeted users to avoid duplicate greetings

2. **[src/robot_pipeline/pipeline.py](../src/robot_pipeline/pipeline.py)**
   - Added `active_users` list to track currently visible users
   - Implemented `get_active_users_info()` to format user data
   - Implemented `get_user_by_name()` for name-based lookups
   - Passes user context to AI agent

3. **[src/robot_pipeline/ai/agent.py](../src/robot_pipeline/ai/agent.py)**
   - Updated `think()` and `think_stream()` to accept `active_users_info`
   - Modified `_build_system_prompt()` to include user context

4. **[src/robot_pipeline/ai/prompts.py](../src/robot_pipeline/ai/prompts.py)**
   - Updated `_base_prompt()` to display camera information
   - Added user awareness to robot's system prompt

## Usage

### Starting the System

```bash
# Terminal 1: Start ROS2 bridge and voice pipeline
cd /home/quanta/Desktop/head-new/FYP-Mihiruth
source venv/bin/activate

# Source ROS2 workspace for custom_interfaces
source /home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.zsh

# Start the robot system
python src/robot_pipeline/ros2_main.py
```

The system will automatically:
1. Subscribe to `/active_users` topic
2. Process incoming camera data
3. Greet new users as they appear
4. Make user information available to the AI agent

### Example Interactions

**Scenario 1: User Detection (Silent)**
```
[Camera detects mihiruth]
[System logs: New user detected: mihiruth (ID: 5)]
[No greeting - continues normal operation]

[Camera detects nidula]
[System logs: New user detected: nidula (ID: 6)]
[No greeting - continues normal operation]
```

**Scenario 2: Wake Word with Personalized Greeting**
```
User: "Hey Quanta"
Robot: "Hello Mihiruth, how can I help you today?"

User: "Do you know my name?"
Robot: "Yes, your name is Mihiruth. How can I assist you?"
```

**Scenario 3: User Inquiry**
```
User: "Who can you see?"
Robot: "I can currently see mihiruth and nidula."
```

**Scenario 4: Name-Based Addressing**
```
User: "Tell nidula about the computer lab"
Robot: "Sure! The computer lab is located on the second floor..."
```

## API Reference

### Pipeline Methods

#### `get_active_users_info() -> str`
Returns formatted string of currently visible users.

**Returns:**
- String with user information or "No users currently visible in camera."

**Example:**
```python
info = pipeline.get_active_users_info()
# "Currently visible users: mihiruth (ID: 5) at angle (horizontal: -29°, vertical: -13°), nidula (ID: 6) at angle (horizontal: 8°, vertical: -1°)"
```

#### `get_user_by_name(name: str) -> People | None`
Retrieves user object by name (case-insensitive).

**Parameters:**
- `name`: User's name to search for

**Returns:**
- `People` object if found, `None` otherwise

**Example:**
```python
user = pipeline.get_user_by_name("mihiruth")
if user:
    print(f"User ID: {user.id}, Angle: {user.hor_angle}°")
```

### ROS2 Bridge Methods

#### `active_users_callback(msg: PeopleArray)`
Processes incoming active user data from camera.

- Updates internal user tracking
- Silently tracks new users (no greeting)
- Removes users no longer visible
- Updates pipeline with current user list

#### `_greet_user(person: People)` [DEPRECATED]
This method is no longer used for automatic greetings. Users are now greeted only on wake word detection with personalized greeting based on camera data.

## Configuration

### Re-enabling Automatic Greetings (If Needed)

If you want to re-enable automatic greetings when users appear:

Edit [src/robot_pipeline/ros2_bridge.py](../src/robot_pipeline/ros2_bridge.py):

```python
# In active_users_callback method, uncomment greeting:
if person.id not in self.greeted_users and person.name:
    self.greeted_users.add(person.id)
    self._greet_user(person)  # Re-enable automatic greeting
    self.get_logger().info(f'👋 New user detected: {person.name} (ID: {person.id})')
```

### Customizing Greeting Message

Edit the greeting in `_greet_user()` method:

```python
def _greet_user(self, person: People):
    """Greet a newly detected user."""
    # Customize this message:
    greeting = f"Welcome {person.name}! How can I help you today?"
    
    if hasattr(self.pipeline, '_loop') and self.pipeline._loop:
        asyncio = __import__('asyncio')
        asyncio.run_coroutine_threadsafe(
            self.pipeline._speak(greeting),
            self.pipeline._loop
        )
```

## Testing

### Run the Demo Script

```bash
python scripts/test_user_detection.py
```

This script demonstrates:
- Sample camera data format
- Expected robot behavior
- Integration status checklist

### Testing with Real Camera

1. Ensure your camera node is publishing to `/active_users`
2. Verify message format matches `custom_interfaces/msg/PeopleArray`
3. Start the robot system
4. Walk into camera frame and observe greeting
5. Ask "Who can you see?" to verify AI integration

### Debugging

Check if camera data is being received:
```bash
ros2 topic echo /active_users
```

View robot logs:
```bash
# Watch for these log messages:
# "👋 New user detected: [name] (ID: [id])"
# "👋 User left: [name] (ID: [id])"
```

## Troubleshooting

### Issue: Users not being greeted

**Check:**
1. Camera node is publishing to `/active_users`
2. Message type matches `custom_interfaces/msg/PeopleArray`
3. User names are non-empty strings
4. ROS2 subscriber is active (check logs)

### Issue: "Could not greet user - pipeline loop not available"

**Solution:** Ensure the pipeline's event loop is properly initialized before ROS2 callbacks are triggered. This warning appears if a user is detected before the voice pipeline is fully started.

### Issue: Duplicate greetings

**Solution:** The system tracks greeted users by ID. If you're seeing duplicates, check that user IDs remain consistent across camera frames.

## Future Enhancements

Potential improvements for this feature:

1. **Farewell Messages**: Add goodbye messages when users leave
2. **Position Awareness**: Use angle data to determine which user is speaking
3. **User Preferences**: Remember user preferences from previous interactions
4. **Face-Based Attention**: Turn to face the speaking user based on angles
5. **Multi-User Conversations**: Handle group conversations with multiple users

## Related Documentation

- [ROS2 Bridge Documentation](RAG_GUIDE.md)
- [Emotion System](EMOTION_SYSTEM.md)
- [Robot Pipeline Architecture](ARCHITECTURE.md)

