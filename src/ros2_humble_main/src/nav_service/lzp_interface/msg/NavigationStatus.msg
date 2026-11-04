# NavigationStatus.msg
int32 waypoint_id
string waypoint_name
string status  # "started", "navigating", "arrived", "failed"
float64 distance_to_goal
float64 current_x
float64 current_y
float64 target_x
float64 target_y
float64 target_theta