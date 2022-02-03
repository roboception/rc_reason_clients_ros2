# This represents a detected load carrier with it's filling level

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

geometry_msgs/PoseStamped pose

# True if a detected load carrier is overfilled. Ignored on setting load carrier.
bool overfilled

# Filling level of the whole load carrier
CellFillingLevel overall_filling_level

# Filling level of each load carrier cell
CellFillingLevel[] cells_filling_levels

GridSize filling_level_cell_count
