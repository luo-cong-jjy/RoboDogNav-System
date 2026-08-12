# Typed state of the active warehouse inspection mission.
std_msgs/Header header
string mission_id
string state
uint32 step_index
uint32 step_count
string step_type
string step_name
string active_floor
uint64 map_generation
bool paused
bool fault
string message
