# An identifier for this particular grasp action
int32 id

# the grasp to perform
moveit_msgs/Grasp grasp 

# the name of the effector link (the one which the grasp refers to).
# This is needed to check whether according to /tf transforms,
# the link is at the right position (grasp.reach_pose) in order to
# accept the grasp. 
string effector_link_name

