#include <InverseKinematics.h>
#include <InverseKinematicsNode.h>

#include <cmath>
#include <limits>
#include <type_traits>

InverseKinematics::InverseKinematics(InverseKinematicsNode* IKNode_)
{
    this->IKNode = IKNode_;
}