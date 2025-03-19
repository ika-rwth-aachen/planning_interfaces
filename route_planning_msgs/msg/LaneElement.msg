geometry_msgs/Pose reference_pose           # pose on reference path of lane

LaneBoundary left_boundary                  # left boundary of lane (optional)
bool has_left_boundary                      # whether left_boundary is valid

LaneBoundary right_boundary                 # right boundary of lane (optional)
bool has_right_boundary                     # whether right_boundary is valid

uint8 speed_limit                           # speed limit [km/h] (0=unlimited)
uint8[] regulatory_element_idx              # indices of regulatory elements from corresponding RouteElement belonging to this LaneElement
bool has_regulatory_elements                # whether regulatory_element_idx is valid

uint8 following_lane_idx                    # index of following LaneElement in following RouteElement
bool has_following_lane_idx                 # whether following_lane_idx is valid