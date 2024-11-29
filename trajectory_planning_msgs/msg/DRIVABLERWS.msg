###########################################################################################################################################################
#   DRIVABLE Trajectory for bicycle model with rear wheel steering (DRIVABLERWS)
###########################################################################################################################################################
uint8 TYPE_ID=2
uint8 STATE_DIM=10

#   sample points of DRIVABLERWS trajectories have the following entries
#   state information for each sample point:
uint8 T=0                           # time to reach sample point [s]
uint8 X=1                           # x position of sample point in trajectory header frame [m]
uint8 Y=2                           # y position of sample point in trajectory header frame [m]
uint8 V=3                           # absolute velocity in vehicle frame [m/s]
uint8 THETA=4                       # heading at sample point in trajectory header frame [rad] from [-pi, pi]
uint8 A=5                           # absolute acceleration in vehicle frame [m/s^2]
uint8 BETA=6                        # slip angle of the vehicle in center of gravity [rad] from [-pi, pi]
uint8 DELTAFRONT=7                  # steering angle of virtual front wheel [rad] from [-pi, pi]
uint8 DELTAREAR=8                   # steering angle of virtual rear wheel [rad] from [-pi, pi]
uint8 S=9                           # driven distance until sample point is reached in [m]
###########################################################################################################################################################
