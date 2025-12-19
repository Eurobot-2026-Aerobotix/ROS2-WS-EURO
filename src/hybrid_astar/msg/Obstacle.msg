# Single obstacle (crate)
string id                    # Unique ID (e.g., "yellow_crate_01")
string color                 # "yellow" or "blue"
string type                  # "horizontal" or "vertical"
geometry_msgs/Pose2D pose    # Center position (x, y, theta in radians)

# Dimensions
float32 width_mm
float32 height_mm

builtin_interfaces/Time detected_at
