
Header header

# Robot state
float64[16] gripper_pose
float64[6] gripper_twist
float64[16] gripper_pose_cmd
float64[] joint_pos
float64[] joint_vel
float64[] joint_eff
float64 gripper_cmd # value in [0,1] (0: closed, 1: open)

# Manipulated object state
string gripped_object_label
float64[16] gripped_object_pose

# Entity information
string[] entity_keys
uint16[] entity_ids
string[] entity_labels
float64[] entity_poses # mod 16

# Predicates
string[] predicate_names
bool[] predicate_values

# Features
string[] twist_from
string[] twist_to
float64[] twists # mod 6


