uint8 TYPE_UNKNOWN=0
uint8 TYPE_YIELD=1
uint8 TYPE_STOP=2
uint8 TYPE_TRAFFIC_LIGHT=3
uint8 TYPE_RIGHT_OF_WAY=4
uint8 TYPE_SPEED_LIMIT=5
uint8 type

uint8 META_VALUE_UNKNOWN=0
# meta values for YIELD, STOP, TRAFFIC_LIGHT
uint8 META_VALUE_MOVEMENT_ALLOWED=1     # movement is allowed
uint8 META_VALUE_MOVEMENT_RESTRICTED=2  # movement is restricted
# meta values for SPEED_LIMIT [km/h]
uint8 META_VALUE_SPEED_UNKNOWN=0        # speed limit is unknown
uint8 META_VALUE_SPEED_UNLIMITED=255    # no speed limit
uint8 meta_value

bool has_validity_stamp                 # whether validity_stamp is set
builtin_interfaces/Time validity_stamp  # regulatory element is valid until this time (e.g., signal chage of traffic light)

geometry_msgs/Point[2] reference_line   # reference/effect line of regulatory element (e.g., stop line)
geometry_msgs/Point[] positions         # position of signals/signs indicating the regulatory element (e.g., positions of traffic lights)