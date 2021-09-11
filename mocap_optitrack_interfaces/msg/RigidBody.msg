# Message for a RigidBody of the NatNet server
# Unique ID of the RigidBody, assigned by the the server
int64 id
# true if the rigid body was correctly tracked, false otherwise
bool valid
# mean error of the estimate
float64 mean_error
#Pose of the body
geometry_msgs/PoseStamped pose_stamped