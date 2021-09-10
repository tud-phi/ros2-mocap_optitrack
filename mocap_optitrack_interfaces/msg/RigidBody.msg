# Message for a RigidBody of the NatNet server
# Unique ID of the RigidBody, assigned by the the server
int64 id
# true if the rigid body was correctly tracked, false otherwise
bool valid
# mean error of the estimate
float64 mean_error
# Position of the body
Point p
#Orientation of the body as unit quaternion
UnitQuaternion q