LaneBoundary left_boundary                  # left boundary of lane (optional)
bool has_left_boundary                      # whether left_boundary is valid

geometry_msgs/Pose reference_pose           # pose on reference path of lane

LaneBoundary right_boundary                 # right boundary of lane (optional)
bool has_right_boundary                     # whether right_boundary is valid

uint8 speed_limit                           # speed limit [km/h] (0=unlimited)
RegulatoryElement[] regulatory_elements     # regulatory elements belonging to this LaneElement

uint8 following_lane_idx                    # index of following LaneElement in following RouteElement
bool has_following_lane_idx                 # whether following_lane_idx is valid