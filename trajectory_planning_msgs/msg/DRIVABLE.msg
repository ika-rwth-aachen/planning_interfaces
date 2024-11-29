###########################################################################################################################################################
#   DRIVABLE Trajectory for bicycle model (ackermann steering)
###########################################################################################################################################################
uint8 TYPE_ID=1
uint8 STATE_DIM=9

#   sample points of DRIVABLE trajectories have the following entries
#   state information for each sample point:
uint8 T=0                           # time to reach sample point [s]
uint8 X=1                           # x position of sample point in trajectory header frame [m]
uint8 Y=2                           # y position of sample point in trajectory header frame [m]
uint8 V=3                           # longitudinal (="absolute", according to ackermann) velocity in vehicle frame [m/s]
uint8 THETA=4                       # heading at sample point in trajectory header frame [rad] from [-pi, pi]
uint8 A=5                           # longitudinal (="absolute", according to ackermann) acceleration in vehicle frame [m/s^2]
uint8 KAPPA=6                       # curvature of trajectory at sample point in [1/m]
uint8 DKAPPA=7                      # curvature derivative of trajectory at sample point in [1/(m*s)]
uint8 S=8                           # driven distance until sample point is reached in [m]
###########################################################################################################################################################
