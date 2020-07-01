# This represents a load carrier by its geometry and its pose

string id

# Required
Box outer_dimensions

# Required
Box inner_dimensions

# Optional. If not provided, the rim thickness is computed from the outer and inner dimensions
Rectangle rim_thickness

# When setting a load carrier, this is the prior if frame_id is not empty (should be 'camera' or 'external')
# For detected load carriers, this is the detected pose
geometry_msgs/PoseStamped pose

# True if a detected load carrier is overfilled. Ignored on setting load carrier.
bool overfilled
