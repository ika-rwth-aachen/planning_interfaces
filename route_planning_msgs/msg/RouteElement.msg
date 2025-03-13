LaneElement[] lane_elements             # cross sections of lanes in driving direction (from left to right)

uint8 suggested_lane_idx                # index of suggested lane within lane_elements
bool has_changed_suggested_lane         # whether suggested path has changed lanes from previous to this RouteElement

DrivableSpaceElement drivable_space     # boundaries of drivable space

float64 s                               # accumulated distance from start of route