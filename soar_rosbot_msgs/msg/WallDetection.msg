# WallDetection.msg

bool front      # Wall detected in front of the robot
bool left       # Wall detected to the left of the robot
bool right      # Wall detected to the right of the robot
bool back       # Wall detected behind the robot

float32 front_distance    # Distance to front wall in meters (0.0 if no wall detected)
float32 left_distance     # Distance to left wall in meters (0.0 if no wall detected)
float32 right_distance    # Distance to right wall in meters (0.0 if no wall detected)
float32 back_distance     # Distance to back wall in meters (0.0 if no wall detected)
