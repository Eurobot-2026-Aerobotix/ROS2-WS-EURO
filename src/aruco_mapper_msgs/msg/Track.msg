int32 track_id
string track_code
int32 aruco_id
string marker_type  # "A" | "B" | "U"

geometry_msgs/Point cam_pos
geometry_msgs/Vector3 cam_vel

bool has_map
geometry_msgs/Point map_pos
geometry_msgs/Vector3 map_vel

bool has_map_yaw
float32 map_yaw_deg