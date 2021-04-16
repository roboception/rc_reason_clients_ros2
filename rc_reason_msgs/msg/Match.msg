# id of template which was matched
string template_id

# unique id of this match
string uuid

geometry_msgs/PoseStamped pose

# associated grasps
string[] grasp_uuids

# how well the object matches the template (-1 if not available)
float64 score
