# User-Specific Chat History Feature

## Overview

The robot now maintains **separate conversation histories for each user** detected by the camera. This prevents confusion and ensures each person has their own contextual conversation that persists when they return.

## How It Works

### Automatic Context Switching (Name-Based)

The robot uses **user names** (not camera IDs) to track conversation histories. This is important because camera IDs can change when users re-enter the frame, but names remain consistent.

```
[User "mihiruth" (ID: 5) enters frame]
Robot switches to "mihiruth"'s history
└─> Previous conversations with mihiruth are restored

[User "mihiruth" leaves, "nidula" (ID: 6) enters]
Robot saves "mihiruth"'s history
Robot switches to "nidula"'s history  
└─> nidula starts with fresh context

[User "nidula" leaves, "mihiruth" returns (now ID: 7!)]
Robot saves "nidula"'s history
Robot switches back to "mihiruth"'s history
└─> mihiruth's previous context is restored (despite new ID!)
```

**Key Insight**: Even if the camera assigns a new ID (5 → 7), the robot recognizes "mihiruth" by name and restores the correct history.

## Example Scenario

### Session with Two Users

```
1. MIHIRUTH ENTERS (ID: 5, name: "mihiruth")
   User: "Hey Quanta"
   Robot: "Hello Mihiruth, how can I help you today?"
   [Active user: 'mihiruth', history length: 0]
   
   User: "Where is the computer lab?"
   Robot: "The computer lab is on the second floor..."
   [Conversation saved to 'mihiruth' history]

2. MIHIRUTH LEAVES, NIDULA ENTERS (ID: 6, name: "nidula")
   ['mihiruth' history saved]
   User: "Hey Quanta"
   Robot: "Hello Nidula, how can I help you today?"
   [Active user: 'nidula', history length: 0]
   
   User: "What is your name?"
   Robot: "My name is Quanta..."
   [Conversation saved to 'nidula' history]

3. NIDULA LEAVES, MIHIRUTH RETURNS (ID: 9, name: "mihiruth")
   ['nidula' history saved]
   [Active user: 'mihiruth', history length: 2]
   ⚠️ Note: Camera gave new ID (9), but name is still "mihiruth"!
   
   User: "Where was that lab again?"
   Robot: "The computer lab is on the second floor..."
   [Robot remembers previous context by NAME, not ID!]
```

## Technical Implementation

### AI Agent Changes

**New Data Structures:**
```python
conversation_histories = {}  # user_name -> conversation history (name, not ID!)
current_user_name = None     # Currently active user name
```

**New Methods:**
- `set_active_user(user_name)` - Switches conversation context by name
- `clear_all_histories()` - Clears all stored histories

**Why Name Instead of ID?**
Camera systems often assign new IDs when people re-enter the frame, but names remain consistent. Using names ensures conversation history persists across ID changes.

### Pipeline Changes

**New Data:**
```python
current_user_name = None  # Tracks active user by name
```

**New Method:**
- `_update_active_user()` - Called before processing each query
  - Monitors camera for active users
  - Extracts user's name from camera data
  - Switches to that user's history (by name)
  - Handles ID changes gracefully (same name = same history)
  - Falls back to generic when no users visible

## Key Features

✅ **Isolated Conversations**: Each user has their own context  
✅ **Automatic Switching**: Robot switches histories based on camera  
✅ **Persistent Memory**: Previous conversations restored when users return  
✅ **No Cross-Talk**: User A's questions don't affect User B's context  
✅ **Generic Fallback**: Uses generic history when no users visible  

## Console Output

Watch for these log messages:

```
Switched to user 'mihiruth', history length: 0
Active user changed to: mihiruth (ID: 5)

Switched to user 'nidula', history length: 0  
Active user changed to: nidula (ID: 6)

Switched to user 'mihiruth', history length: 2  ← Same name, history restored!
Active user changed to: mihiruth (ID: 9)      ← New ID, but name matched!

No users visible - using generic conversation history
```

**Key Point**: Notice how "mihiruth" got ID 9 instead of 5, but the robot still found the history because it matches by name!

## Testing

Run the demo:
```bash
python scripts/test_user_history.py
```

Test with real camera:
1. Person A enters and asks questions
2. Person A leaves, Person B enters and asks different questions  
3. Person B leaves, Person A returns
4. Person A asks follow-up questions
5. **Robot should remember Person A's previous context!**

## Benefits

### For Users
- More natural conversations
- Robot remembers what you discussed
- No confusion with other users' questions
- Personalized experience

### For Development
- Clean separation of user contexts
- Automatic history management
- Easy to debug per-user interactions
- Scalable to many users

## Implementation Files

- ✅ [src/robot_pipeline/ai/agent.py](../src/robot_pipeline/ai/agent.py)
  - `conversation_histories` dictionary
  - `set_active_user()` method
  - `clear_all_histories()` method

- ✅ [src/robot_pipeline/pipeline.py](../src/robot_pipeline/pipeline.py)
  - `current_user_id` tracking
  - `_update_active_user()` method
  - Automatic switching on queries

- ✅ [scripts/test_user_history.py](../scripts/test_user_history.py)
  - Demonstration script

## Related Documentation

- [Camera Integration Guide](../docs/CAMERA_INTEGRATION.md)
- [Camera Integration Summary](../CAMERA_INTEGRATION_SUMMARY.md)
- [Quick Start Guide](../QUICK_START_CAMERA.md)

---

**Status:** ✅ IMPLEMENTED  
**Date:** February 10, 2026  
**Feature:** User-specific chat history management
