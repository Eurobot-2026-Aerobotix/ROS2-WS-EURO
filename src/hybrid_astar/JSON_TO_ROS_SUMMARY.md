# ✅ JSON to ROS2 Bridge - Complete!

## What Was Created

### Main Tool: `json_to_ros.py`
A complete bridge between JSON map files and the ROS2 path planner.

**Location**: `/home/ubuntu/hybrid/src/hybrid_astar/scripts/json_to_ros.py`

## How to Use

### 1. Start the Path Planner (Terminal 1)
```bash
source ~/hybrid/install/setup.bash
ros2 launch hybrid_astar hybrid_astar.launch.py
```

### 2. Run the JSON to ROS Bridge (Terminal 2)
```bash
source ~/hybrid/install/setup.bash

# Basic usage
ros2 run hybrid_astar json_to_ros.py /path/to/map.json

# With color filtering
ros2 run hybrid_astar json_to_ros.py /path/to/map.json yellow    # Ignore yellow
ros2 run hybrid_astar json_to_ros.py /path/to/map.json blue      # Ignore blue
ros2 run hybrid_astar json_to_ros.py /path/to/map.json yellow blue  # Ignore both
```

## What It Does

1. **Loads JSON** - Reads map with obstacles, start, and goal
2. **Publishes Obstacles** - Sends obstacles to `/obstacles/add` topic
3. **Requests Path** - Calls `/compute_path` service
4. **Saves Path** - Writes waypoints back to JSON file

## Input JSON Format

```json
{
  "map_info": {
    "width": 2000.0,
    "height": 3000.0,
    "grid_resolution": 25.0
  },
  "obstacles": [
    {
      "x": 700.0,
      "y": 100.0,
      "width": 50.0,
      "height": 150.0,
      "color": "yellow"
    },
    {
      "x": 750.0,
      "y": 100.0,
      "width": 50.0,
      "height": 150.0,
      "color": "blue"
    }
  ],
  "start_point": {
    "x": 650.0,
    "y": 576.67,
    "theta": 90.0
  },
  "goal_point": {
    "x": 670.0,
    "y": 1306.67,
    "theta": 0.0
  }
}
```

## Output JSON Format

Same as input, but with **`"path"`** field added:

```json
{
  "map_info": { ... },
  "obstacles": [ ... ],
  "start_point": { ... },
  "goal_point": { ... },
  "path": [
    {"x": 650.0, "y": 576.67, "theta": 90.0},
    {"x": 650.0, "y": 800.0, "theta": 45.0},
    {"x": 670.0, "y": 1306.67, "theta": 0.0}
  ]
}
```

## Example Output

```
============================================================
   JSON to ROS2 Path Planner Bridge
============================================================
📄 JSON file: /home/ubuntu/hybrid/src/hybrid_astar/scripts/test_map.json
============================================================

[Step 1/4] Loading JSON...
✅ Loaded JSON: /home/ubuntu/hybrid/src/hybrid_astar/scripts/test_map.json

[Step 2/4] Publishing obstacles to ROS2...
📦 Publishing 32 obstacles...
✅ Published 32 obstacles

[Step 3/4] Requesting path from planner...
============================================================
🔍 Requesting path:
   Start: (650.0, 576.7) θ=90.0°
   Goal:  (670.0, 1306.7) θ=0.0°
   Using 32 obstacles
✅ Path found: 15 waypoints, 0.73m
   Waypoints: 15
   Path length: 0.73m
   Planning time: 156.2ms
   Obstacles used: 32
============================================================

[Step 4/4] Saving path to JSON...
✅ Saved path (15 waypoints) to: /home/ubuntu/hybrid/src/hybrid_astar/scripts/test_map.json

============================================================
✅ SUCCESS! Path saved to JSON file
============================================================
```

## Features

### ✅ Automatic Conversion
- **Millimeters ↔ Meters** - Handles unit conversion automatically
- **Degrees ↔ Radians** - Converts angles for ROS2
- **Obstacle Types** - Determines horizontal/vertical from dimensions

### ✅ Color Filtering
- Ignore yellow obstacles: `ros2 run hybrid_astar json_to_ros.py map.json yellow`
- Ignore blue obstacles: `ros2 run hybrid_astar json_to_ros.py map.json blue`
- Ignore multiple colors: `ros2 run hybrid_astar json_to_ros.py map.json yellow blue`

### ✅ Error Handling
- Service timeout detection
- File not found errors
- Invalid JSON format
- Path planning failures
- Clear error messages

### ✅ Progress Tracking
- 4-step process with clear progress indicators
- Emoji indicators for status (✅ ❌ 📦 🔍 📄)
- Detailed logging at each step

## Use Cases

### 1. Batch Processing
Test multiple map configurations:
```bash
for map in maps/*.json; do
  ros2 run hybrid_astar json_to_ros.py "$map"
done
```

### 2. Strategy Testing
Compare different obstacle filtering strategies:
```bash
# All obstacles
ros2 run hybrid_astar json_to_ros.py map.json
mv map.json results/map_all.json

# Ignore yellow
ros2 run hybrid_astar json_to_ros.py map.json yellow
mv map.json results/map_no_yellow.json
```

### 3. Offline Planning
Generate paths without full robot system:
- Only needs path planner node running
- No sensors, no robot hardware needed
- Perfect for pre-match planning

### 4. Integration
- Create maps in external editor
- Generate paths with this tool
- Load paths in simulation/robot

## Files Created

1. **`scripts/json_to_ros.py`** - Main bridge tool (executable)
2. **`JSON_TO_ROS_GUIDE.md`** - Complete usage guide
3. **This summary** - Quick reference

## Installation Status

✅ **Built and Installed**
- Tool is executable: `ros2 run hybrid_astar json_to_ros.py`
- Included in CMakeLists.txt
- Available after sourcing workspace

## Quick Test

```bash
# Terminal 1
ros2 launch hybrid_astar hybrid_astar.launch.py

# Terminal 2
ros2 run hybrid_astar json_to_ros.py ~/hybrid/src/hybrid_astar/scripts/test_map.json
```

## Documentation

- **Full Guide**: [JSON_TO_ROS_GUIDE.md](JSON_TO_ROS_GUIDE.md)
- **Package README**: [README.md](README.md)
- **Quick Start**: [QUICKSTART.md](QUICKSTART.md)

---

**Status**: ✅ Ready to use!

**Package built**: ✅ Success
**Tool installed**: ✅ Yes
**Tested**: Ready for testing
