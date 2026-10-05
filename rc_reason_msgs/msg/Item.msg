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
string BOX=BOX
string TEXTURED_BOX=TEXTURED_BOX
string BAG=BAG
string CONSUMER_GOODS=CONSUMER_GOODS
string SHEET_METAL=SHEET_METAL
string MAIL_ITEMS=MAIL_ITEMS

# If type is RECTANGLE or TEXTURED_RECTANGLE, i.e. if dimensioning was not requested
Rectangle rectangle

# If type is BOX or TEXTURED_BOX, i.e. if dimensioning was requested
Box box

# If type is BAG, CONSUMER_GOODS, MAIL_ITEMS or SHEET_METAL
# If dimensioning was requested, the bounding box is aligned with the reference plane
Box bounding_box

# The pose of the item
geometry_msgs/PoseStamped pose

# If type is TEXTURED_RECTANGLE or TEXTURED_BOX. ID of the template item was matched to
string template_id

# If type is TEXTURED_RECTANGLE or TEXTURED_BOX. ID of the view this item was matched to
string view_uuid

# If type is TEXTURED_RECTANGLE or TEXTURED_BOX. User-defined name of the view this item was matched to
string view_name

# If type is TEXTURED_RECTANGLE or TEXTURED_BOX. Gives whether the view has an offset defined
bool view_pose_set
