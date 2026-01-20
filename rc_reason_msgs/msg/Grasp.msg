# Definition of a grasp

# user defined id (name)
string id

# unique id of this grasp
string uuid

# unique id of the match this grasp is associated to
string match_uuid

geometry_msgs/PoseStamped pose

# Optional
int8 priority

# Optional. If collision checked, the ID of the gripper used.
string gripper_id

# Optional. stroke per finger for the FINGER elements of the given gripper ID while approaching
float64 stroke_per_finger_approach_mm

# Optional. stroke per finger for the FINGER elements of the given gripper ID while grasping
float64 stroke_per_finger_grasp_mm

bool collision_checked
