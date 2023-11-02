# This represents a load carrier by its geometry and its pose

string id

# Optional. The load carrier rim model. The only supported value is STANDARD
string type

# Required
Box outer_dimensions

# Required
Box inner_dimensions

# Optional. If not provided, the rim thickness is computed from the outer and inner dimensions
Rectangle rim_thickness

# Optional. Height of the rim step for rims with an L-profile.
float64 rim_step_height

# Optional. Length of the rim ledge in x and y direction. This requires rim_thickness to be set.
Rectangle rim_ledge

# Optional. Height of the open side of a 3-sided load carrier measured from the outer bottom.
float64 height_open_side

# When setting a load carrier, this is the prior if frame_id is not empty (should be 'camera' or 'external')
geometry_msgs/PoseStamped pose

# Optional. Indicates how to interpret the load carrier pose
# If not set and pose is zero, it's equivalent to NO_POSE
# If not set and any value in pose is not zero, it's equivalent to ORIENTATION_PRIOR
# Valid values are:
# NO_POSE
# EXACT_POSE
# ORIENTATION_PRIOR
string pose_type
