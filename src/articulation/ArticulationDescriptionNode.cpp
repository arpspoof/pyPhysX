#include "ArticulationDescriptionNode.h"

using namespace physx;
using namespace std;

static PxQuat rtz(PxPi / 2, PxVec3(0, 0, 1));
static PxQuat rtzinv(-PxPi / 2, PxVec3(0, 0, 1));

static PxTransform GetJointPose(PxVec3 offset) 
{
	return PxTransform(rtzinv.rotate(offset));
}

ArticulationDescriptionNode::ArticulationDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    :linkName(linkName), jointName(jointName), body(body),
    posOffsetJointToParentJoint(posOffsetJointToParentJoint),
    posOffsetLinkToInboundJoint(posOffsetLinkToInboundJoint) 
{
}

Link* ArticulationDescriptionNode::CreateLink(Articulation& ar, Link *parentLink,
	PxVec3 parentJointPos, PxVec3 parentLinkPos) const 
{
	PxVec3 jointPos = parentJointPos + posOffsetJointToParentJoint;
	PxVec3 linkPos = jointPos + posOffsetLinkToInboundJoint;
	Link *link = ar.AddLink(linkName, parentLink, PxTransform(linkPos, rtz), body);
	link->globalPositionOffset = linkPos;
	if (parentLink) {
		Joint *joint = CreateJoint(ar, link,
			GetJointPose(jointPos - parentLinkPos),
			GetJointPose(jointPos - linkPos)
		);
		link->inboundJoint = joint;
		joint->globalPositionOffset = jointPos;
		joint->childLink = link;
		joint->parentLink = parentLink;
	}
	return link;
}

NULLDescriptionNode::NULLDescriptionNode(string linkName, NULLLinkBody *body)
    : ArticulationDescriptionNode(linkName, "", body,
        PxVec3(0, 0, 0), PxVec3(0, 0, 0)) 
{
}

Joint* NULLDescriptionNode::CreateJoint(Articulation&, Link *,
    PxTransform,
    PxTransform) const 
{
    return NULL;
}

FixedDescriptionNode::FixedDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    : ArticulationDescriptionNode(linkName, jointName, body,
        posOffsetLinkToInboundJoint, posOffsetJointToParentJoint) 
{
}

Joint* FixedDescriptionNode::CreateJoint(Articulation& ar, Link *link,
	PxTransform parentPose,
	PxTransform childPose) const 
{
	return ar.AddFixedJoint(jointName, link, parentPose, childPose);
}

SpericalDescriptionNode::SpericalDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    : ArticulationDescriptionNode(linkName, jointName, body,
        posOffsetLinkToInboundJoint, posOffsetJointToParentJoint) 
{
}

Joint* SpericalDescriptionNode::CreateJoint(Articulation& ar, Link *link,
	PxTransform parentPose,
	PxTransform childPose) const 
{
	return ar.AddSpericalJoint(jointName, link, parentPose, childPose);
}

RevoluteDescriptionNode::RevoluteDescriptionNode(string linkName, string jointName, LinkBody *body,
    PxArticulationAxis::Enum axis,
    PxVec3 posOffsetLinkToInboundJoint, PxVec3 posOffsetJointToParentJoint)
    : ArticulationDescriptionNode(linkName, jointName, body,
        posOffsetLinkToInboundJoint, posOffsetJointToParentJoint), axis(axis) 
{
}

Joint* RevoluteDescriptionNode::CreateJoint(Articulation& ar, Link *link,
	PxTransform parentPose,
	PxTransform childPose) const 
{
	return ar.AddRevoluteJoint(jointName, link, axis, parentPose, childPose);
}
