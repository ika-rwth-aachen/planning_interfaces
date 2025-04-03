LaneElement[] lane_elements             # cross sections of lanes in driving direction (from left to right)

geometry_msgs/Point left_boundary       # left boundary of drivable space
geometry_msgs/Point right_boundary      # right boundary of drivable space

RegulatoryElement[] regulatory_elements # regulatory elements belonging to this RouteElement (refering at least one LaneElement)

uint8 suggested_lane_idx                # index of suggested lane within lane_elements
bool will_change_suggested_lane         # whether suggested path will change lanes from this RouteElement to the next

float64 s                               # accumulated distance from start of route