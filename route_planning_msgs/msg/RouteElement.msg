LaneElement[] lane_elements             # cross sections of lanes in driving direction (from left to right)

uint8 suggested_lane_idx                # index of suggested lane within lane_elements
bool will_change_suggested_lane         # whether suggested path will change lanes from this RouteElement to the next

float64 s                               # accumulated distance from start of route [m]

bool is_enriched                        # whether this RouteElement is enriched with lane boundaries, adjacent lanes, drivable space, regulatory elements

geometry_msgs/Point left_boundary       # left boundary of drivable space (only if is_enriched)
geometry_msgs/Point right_boundary      # right boundary of drivable space (only if is_enriched)

RegulatoryElement[] regulatory_elements # regulatory elements belonging to this RouteElement (referring at least one LaneElement) (only if is_enriched)