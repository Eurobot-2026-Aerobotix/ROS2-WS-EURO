# Result JSON Generation

## Overview
The `yaml_to_ros.py` script has been enhanced to automatically generate a `result.json` file after receiving a path from the path planner. This JSON file follows the same format as `test_map.json`.

## Features

### Generated JSON Structure
The `result.json` file contains:

1. **map_info**: Fixed map configuration
   - width: 2000.0 mm
   - height: 3000.0 mm
   - grid_resolution: 25.0 mm
   - window dimensions: 1200x800

2. **obstacles**: Converted from YAML batches
   - Each member in a batch becomes an obstacle
   - Position (x, y) from `map_pos`
   - Dimensions based on direction:
     - Vertical: 50mm width × 150mm height
     - Horizontal: 150mm width × 50mm height
   - Color from member color

3. **start_point**: From YAML file
   - x, y coordinates (in mm)
   - theta (orientation in degrees)

4. **goal_point**: From YAML file
   - x, y coordinates (in mm)
   - theta (orientation in degrees)

5. **path**: Extracted from path planner response
   - Array of waypoints with x, y, theta
   - Coordinates converted from meters to millimeters
   - Theta converted from quaternion to degrees

## Usage

### Basic Usage (default output file)
```bash
ros2 run hybrid_astar yaml_to_path_planner.py test_aruco.yaml
```
This will generate `result.json` in the current directory.

### Custom Output File
```bash
ros2 run hybrid_astar yaml_to_path_planner.py test_aruco.yaml custom_result.json
```

### Expected Output
```
📦 YAML → Pathfinder node started
⏳ Waiting for /compute_path service...
✅ /compute_path service available
✅ Loaded YAML (6 batches)
✅ Converted 24 obstacles
🚀 Published 6 batches (24 obstacles)
🧭 Requesting path...
✅ Path planned successfully
   Waypoints: X
   Length: X.XX m
   Time: X.X ms
✅ Extracted X waypoints from path
✅ Saved result to result.json
```

## Implementation Details

### Key Functions

#### `_convert_batches_to_obstacles()`
Converts YAML batch format to obstacle format:
- Iterates through all batches and their members
- Creates obstacles with proper dimensions based on direction
- Stores color information

#### `_extract_path_from_response(response)`
Extracts path data from ComputePath service response:
- Reads pose array from response
- Converts quaternion orientation to degrees
- Converts meters to millimeters
- Stores in result_data structure

#### `_save_result_json()`
Saves the complete result to JSON file:
- Uses json.dump with indent=2 for readability
- Logs success/failure

## Example Result JSON

```json
{
  "map_info": {
    "width": 2000.0,
    "height": 3000.0,
    "grid_resolution": 25.0,
    "window_width": 1200,
    "window_height": 800
  },
  "obstacles": [
    {
      "x": 725.0,
      "y": 175.0,
      "width": 50.0,
      "height": 150.0,
      "color": "yellow"
    },
    ...
  ],
  "start_point": {
    "x": 200.0,
    "y": 400.0,
    "theta": -90.0
  },
  "goal_point": {
    "x": 400.0,
    "y": 2700.0,
    "theta": 90.0
  },
  "path": [
    {
      "x": 200.0,
      "y": 400.0,
      "theta": -10.0
    },
    ...
  ]
}
```

## Notes

- The path extraction assumes the response has a `path` attribute containing poses
- If the path format differs, you may need to adjust `_extract_path_from_response()`
- Coordinates are automatically converted from meters (ROS) to millimeters (JSON)
- Angles are converted from quaternions to degrees
