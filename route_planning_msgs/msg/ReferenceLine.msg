# A compact global reference line.
#
# The origin is the first point of the line. Each delta point is relative to
# the origin, not to the preceding point, so reconstruction errors cannot
# accumulate along the line.

std_msgs/Header header
Point2D origin
Point2D32[] delta_points
