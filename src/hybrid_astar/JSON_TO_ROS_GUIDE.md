# JSON to ROS2 Bridge Tool

## Overview

The `json_to_ros.py` tool allows you to:
1. Load obstacles and start/goal from a JSON file
2. Publish obstacles to the ROS2 path planner
3. Request a path computation
4. Save the computed path back to the JSON file

This is perfect for **batch processing**, **testing**, or **integrating with external map editors**.

## Usage

### Basic Usage

```bash
# Terminal 1: Start the path planner node
source ~/hybrid/install/setup.bash
ros2 launch hybrid_astar hybrid_astar.launch.py

# Terminal 2: Run json_to_ros with your JSON file
source ~/hybrid/install/setup.bash
ros2 run hybrid_astar json_to_ros.py /path/to/your/map.json
```

### With Color Filtering

```bash
# Ignore yellow obstacles
ros2 run hybrid_astar json_to_ros.py map.json yellow

# Ignore blue obstacles
ros2 run hybrid_astar json_to_ros.py map.json blue

# Ignore both colors (no obstacles)
ros2 run hybrid_astar json_to_ros.py map.json yellow blue
```

## JSON File Format

### Input Format (Before Path Generation)

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

### Output Format (After Path Generation)

The tool will add a `"path"` field with waypoints:

```json
{
  "map_info": { ... },
  "obstacles": [ ... ],
  "start_point": { ... },
  "goal_point": { ... },
  "path": [
    {
      "x": 650.0,
      "y": 576.67,
      "theta": 90.0
    },
    {
      "x": 650.0,
      "y": 800.0,
      "theta": 45.0
    },
    {
      "x": 670.0,
      "y": 1306.67,
      "theta": 0.0
    }
  ]
}
```

## Required JSON Fields

### `map_info` (optional, for documentation)
- `width`: Map width in mm
- `height`: Map height in mm  
- `grid_resolution`: Grid resolution in mm

### `obstacles` (required)
Array of obstacles, each with:
- `x`: X position in mm
- `y`: Y position in mm
- `width`: Width in mm
- `height`: Height in mm
- `color`: "yellow" or "blue"

### `start_point` (required)
- `x`: Start X position in mm
- `y`: Start Y position in mm
- `theta`: Start orientation in degrees

### `goal_point` (required)
- `x`: Goal X position in mm
- `y`: Goal Y position in mm
- `theta`: Goal orientation in degrees

## Workflow Example

### Step 1: Create your map JSON

```json
{
  "map_info": {
    "width": 3000.0,
    "height": 2000.0,
    "grid_resolution": 25.0
  },
  "obstacles": [
    {"x": 1000.0, "y": 500.0, "width": 150.0, "height": 50.0, "color": "yellow"},
    {"x": 1500.0, "y": 800.0, "width": 50.0, "height": 150.0, "color": "blue"}
  ],
  "start_point": {"x": 100.0, "y": 100.0, "theta": 0.0},
  "goal_point": {"x": 2800.0, "y": 1800.0, "theta": 90.0}
}
```

Save as `my_map.json`

### Step 2: Start the planner

```bash
ros2 launch hybrid_astar hybrid_astar.launch.py
```

### Step 3: Generate path

```bash
ros2 run hybrid_astar json_to_ros.py my_map.json
```

Output:
```
============================================================
   JSON to ROS2 Path Planner Bridge
============================================================
📄 JSON file: /home/ubuntu/my_map.json
============================================================

[Step 1/4] Loading JSON...
✅ Loaded JSON: /home/ubuntu/my_map.json

[Step 2/4] Publishing obstacles to ROS2...
📦 Publishing 2 obstacles...
✅ Published 2 obstacles

[Step 3/4] Requesting path from planner...
============================================================
🔍 Requesting path:
   Start: (100.0, 100.0) θ=0.0°
   Goal:  (2800.0, 1800.0) θ=90.0°
   Using 2 obstacles
✅ Path found: 25 waypoints, 2.85m
   Waypoints: 25
   Path length: 2.85m
   Planning time: 125.3ms
   Obstacles used: 2
============================================================

[Step 4/4] Saving path to JSON...
✅ Saved path (25 waypoints) to: /home/ubuntu/my_map.json

============================================================
✅ SUCCESS! Path saved to JSON file
============================================================
```

### Step 4: Check the result

```bash
cat my_map.json
```

The file now contains the `"path"` field with all waypoints!

## Use Cases

### 1. Batch Testing
Test multiple map configurations automatically:

```bash
#!/bin/bash
for map in maps/*.json; do
  echo "Processing $map..."
  ros2 run hybrid_astar json_to_ros.py "$map"
done
```

### 2. Strategy Comparison
Generate paths with different color filters:

```bash
# Path avoiding all obstacles
ros2 run hybrid_astar json_to_ros.py map.json
cp map.json map_all.json

# Path ignoring yellow
ros2 run hybrid_astar json_to_ros.py map.json yellow
cp map.json map_no_yellow.json

# Path ignoring blue
ros2 run hybrid_astar json_to_ros.py map.json blue
cp map.json map_no_blue.json
```

### 3. Integration with External Tools
- Create maps in a GUI editor (exports JSON)
- Use `json_to_ros.py` to generate paths
- Load paths in simulation or robot

### 4. Offline Path Planning
Generate paths without running the full robot system:
1. Start only the path planner node
2. Run `json_to_ros.py` with your map
3. Use the generated path in your mission

## Coordinate System

- **Units**: Millimeters (mm) in JSON
- **Origin**: Bottom-left corner (0, 0)
- **Theta**: Degrees (0° = right, 90° = up)
- **ROS2 Internal**: Meters (automatically converted)

## Error Handling

### Service Not Available
```
❌ Path planning service not available!
```
**Solution**: Make sure the planner node is running:
```bash
ros2 launch hybrid_astar hybrid_astar.launch.py
```

### No Path Found
```
❌ No path found - start/goal in collision or blocked
```
**Solutions**:
- Check if start/goal are inside obstacles
- Verify obstacles aren't blocking all paths
- Adjust obstacle positions or start/goal locations

### File Not Found
```
❌ File not found: /path/to/map.json
```
**Solution**: Use absolute path or run from the correct directory

## Tips

1. **Use absolute paths** or run from the same directory as your JSON files
2. **Check visualization** in RViz while running to see the computed path
3. **Backup your JSON** before running (the tool modifies the file)
4. **Test with simple maps** first before complex scenarios
5. **Use color filtering** strategically for game tactics

## Advanced Usage

### Custom Script
```python
#!/usr/bin/env python3
import subprocess
import json

# Load map
with open('map.json') as f:
    data = json.load(f)

# Generate path
subprocess.run(['ros2', 'run', 'hybrid_astar', 'json_to_ros.py', 'map.json'])

# Use the path
with open('map.json') as f:
    data = json.load(f)
    path = data['path']
    print(f"Generated path with {len(path)} waypoints")
```

### Integration with Python
```python
# After running json_to_ros.py
import json

with open('map.json') as f:
    data = json.load(f)

# Extract path waypoints
for i, waypoint in enumerate(data['path']):
    print(f"Waypoint {i}: ({waypoint['x']:.1f}, {waypoint['y']:.1f}) @ {waypoint['theta']:.1f}°")
```

## See Also

- [README.md](README.md) - Complete package documentation
- [QUICKSTART.md](QUICKSTART.md) - Quick start guide
- Main ROS2 API for real-time operation
