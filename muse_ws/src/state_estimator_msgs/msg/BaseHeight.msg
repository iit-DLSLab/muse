std_msgs/Header header

# Estimated height of the base from the ground
float64 height

# Height calculated from each foot position relative to base
float64 height_lf  # Left front foot
float64 height_rf  # Right front foot  
float64 height_lh  # Left hind foot
float64 height_rh  # Right hind foot

# Contact status for each foot (used in height calculation)
bool stance_lf
bool stance_rf
bool stance_lh
bool stance_rh

# Number of feet in contact used for height estimation
int32 num_feet_in_contact