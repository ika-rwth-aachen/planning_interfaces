uint8 TYPE_UNKNOWN=0
uint8 TYPE_YIELD=1
uint8 TYPE_STOP=2
uint8 TYPE_TRAFFIC_LIGHT=3
uint8 TYPE_RIGHT_OF_WAY=4
uint8 TYPE_SPEED_LIMIT=5
uint8 type

uint8 META_VALUE_STATE_UNKNOWN=0
# meta values for YIELD, STOP, TRAFFIC_LIGHT
uint8 META_VALUE_MOVEMENT_ALLOWED=1     # movement is allowed
uint8 META_VALUE_MOVEMENT_RESTRICTED=2  # movement is restricted
# meta values for SPEED_LIMIT
uint8 META_VALUE_SPEED_1=1              # speed limit of 1km/h
uint8 META_VALUE_SPEED_30=30            # speed limit of 30km/h
uint8 META_VALUE_SPEED_50=50            # speed limit of 50km/h
uint8 META_VALUE_SPEED_70=70            # speed limit of 70km/h
uint8 meta_value

geometry_msgs/Point[2] effect_line      # effect line of regulatory element, e.g., stop line
geometry_msgs/Point[] sign_positions    # position of signs signaling the regulatory element, e.g., positions of traffic lights