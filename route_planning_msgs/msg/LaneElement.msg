geometry_msgs/Pose reference_pose           # pose on reference path of lane

LaneBoundary left_boundary                  # left boundary of lane (only if is_enriched)
LaneBoundary right_boundary                 # right boundary of lane (only if is_enriched)

uint8 speed_limit                           # speed limit [km/h] (0=unknown, 255=unlimited)
uint8[] regulatory_element_idcs             # indices of regulatory elements from corresponding RouteElement belonging to this LaneElement (only if is_enriched)

uint8 following_lane_idx                    # index of following LaneElement in following RouteElement
bool has_following_lane_idx                 # whether following_lane_idx is valid