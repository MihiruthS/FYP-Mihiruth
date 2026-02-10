# 🚀 Quick Start - Camera Integration

## Start the Robot (Easy Way)

```bash
cd /home/quanta/Desktop/head-new/FYP-Mihiruth
./start_robot.sh
```

That's it! The script handles everything automatically.

---

## What Happens When You Run It?

1. ✅ Activates Python virtual environment
2. ✅ Sources ROS2 workspace (`/home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.bash`)
3. ✅ Checks for camera feed on `/active_users` topic
4. ✅ Starts robot voice pipeline with full camera integration

---

## Test Camera Integration

### Check if camera is publishing:
```bash
source /home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.zsh
ros2 topic echo /active_users
```

### Expected output:
```
data:
- id: 5
  name: mihiruth
  hor_angle: -29
  ver_angle: -13
  missed_frames: 0
```

---

## Expected Robot Behavior

### When you enter the camera frame:
```
[System logs: New user detected: mihiruth (ID: 5)]
[No greeting - silent tracking]
```

### When you say "Hey Quanta":
```
Robot: "Hello Mihiruth, how can I help you today?"
```

### When you ask "Who can you see?":
```
Robot: "I can currently see mihiruth and nidula."
```

### When you ask "Do you know my name?":
```
Robot: "Yes, your name is Mihiruth. How can I assist you today?"
```

---

## Manual Start (If Needed)

```bash
cd /home/quanta/Desktop/head-new/FYP-Mihiruth
source venv/bin/activate
source /home/quanta/Desktop/Head-Code-Old/receptionist-3.0-head/install/setup.zsh
python src/robot_pipeline/ros2_main.py
```

---

## Troubleshooting

### "Custom interfaces not found"
- Make sure ROS2 workspace is sourced
- Build custom_interfaces: `colcon build --packages-select custom_interfaces`

### "Camera feed not detected"
- Check if camera node is running
- Verify topic exists: `ros2 topic list | grep active_users`

### "No personalized greeting on wake word"
- Verify camera feed has user names
- Check logs for: "👋 New user detected: [name]"
- User must be visible in frame when wake word is said

### "Robot still greeting automatically"
- Check that code changes were applied
- Restart the robot system
- Verify you're running the updated version

---

## Files Modified

- ✅ [src/robot_pipeline/ros2_bridge.py](src/robot_pipeline/ros2_bridge.py)
- ✅ [src/robot_pipeline/pipeline.py](src/robot_pipeline/pipeline.py)
- ✅ [src/robot_pipeline/ai/agent.py](src/robot_pipeline/ai/agent.py)
- ✅ [src/robot_pipeline/ai/prompts.py](src/robot_pipeline/ai/prompts.py)
- ✅ [src/robot_pipeline/ros2_main.py](src/robot_pipeline/ros2_main.py)

---

**Full Documentation**: [CAMERA_INTEGRATION_SUMMARY.md](CAMERA_INTEGRATION_SUMMARY.md)
