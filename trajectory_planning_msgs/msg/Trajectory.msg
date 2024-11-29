std_msgs/Header header

# to ensure that the type_id is unique, all existing trajectory types and their corresponding id's are listed here:
#   REFERENCE:   0
#   DRIVABLE:    1
#   DRIVABLERWS: 2
uint8 type_id

# flatten sample-point-state matrix according to type (sample points x states)
float64[] states

# quality score of trajectory
uint8 score

# is standstill trajectory
bool standstill