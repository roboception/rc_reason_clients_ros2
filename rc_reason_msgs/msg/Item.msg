# Definition of a detected item

# Required
string uuid

# Contains uuids of grasps on this object (If requested)
string[] grasp_uuids

# Required
string type

# Types
string RECTANGLE=RECTANGLE
string TEXTURED_RECTANGLE=TEXTURED_RECTANGLE
string BAG=BAG
string CONSUMER_GOODS=CONSUMER_GOODS
string SHEET_METAL=SHEET_METAL

# If type is RECTANGLE or TEXTURED_RECTANGLE
Rectangle rectangle

# If type is BAG, CONSUMER_GOODS or SHEET_METAL
Box bounding_box

# The pose of the item
geometry_msgs/PoseStamped pose

# If type is TEXTURED_RECTANGLE. ID of the template item was matched to
string template_id

# If type is TEXTURED_RECTANGLE. ID of the view this item was matched to
string view_uuid

# If type is TEXTURED_RECTANGLE. User-defined name of the view this item was matched to
string view_name

# If type is TEXTURED_RECTANGLE. Gives whether the view has an offset defined
bool view_pose_set
