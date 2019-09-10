#pragma once

#include "PxPhysicsAPI.h"
#include "LinkBody.h"
#include "Articulation.h"

#include <string>
#include <vector>

class ArticulationDescriptionNode {
public:
	std::string linkName;
	std::string jointName;
	LinkBody *body;
	physx::PxVec3 posOffsetLinkToInboundJoint;
	physx::PxVec3 posOffsetJointToParentJoint;
	ArticulationDescriptionNode *parent;
	std::vector<ArticulationDescriptionNode*> children;
protected:
	ArticulationDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
	virtual Joint* CreateJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const = 0;
public:
	Link* CreateLink(Articulation& ar, Link *parentLink,
		physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
};

class NULLDescriptionNode : public ArticulationDescriptionNode {
public:
	NULLDescriptionNode(std::string linkName, NULLLinkBody *body);
	Joint* CreateJoint(Articulation&, Link *,
		physx::PxTransform,
		physx::PxTransform) const override;
};

class FixedDescriptionNode : public ArticulationDescriptionNode {
public:
	FixedDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
	Joint* CreateJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const override;
};

class SpericalDescriptionNode : public ArticulationDescriptionNode {
public:
	SpericalDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
	Joint* CreateJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const override;
};

class RevoluteDescriptionNode : public ArticulationDescriptionNode {
	physx::PxArticulationAxis::Enum axis;
public:
	RevoluteDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
		physx::PxArticulationAxis::Enum axis,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
	Joint* CreateJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const override;
};
