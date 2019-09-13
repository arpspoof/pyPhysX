#pragma once

#include "PxPhysicsAPI.h"
#include "LinkBody.h"
#include "Actor.h"

#include <vector>

class Link;

class Joint {
public:
	Link *parentLink;
	Link *childLink;
	int nDof;
	int cacheIndex;
	physx::PxArticulationJointReducedCoordinate *joint;
	physx::PxVec3 globalPositionOffset;
protected:
	Joint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
};

class FixedJoint : public Joint {
public:
	FixedJoint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
};

class SphericalJoint : public Joint {
public:
	SphericalJoint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
};

class RevoluteJoint : public Joint {
public:
	physx::PxArticulationAxis::Enum axis;
	RevoluteJoint(Link *link, physx::PxArticulationAxis::Enum axis,
		physx::PxTransform parentPose, physx::PxTransform childPose);
};

class Link :public RigidActor {
public:
	Link *parentLink;
	std::vector<Link*> childLinks;
	physx::PxArticulationLink *link;
	Joint *inboundJoint;
	physx::PxVec3 globalPositionOffset;
	Link(physx::PxArticulationReducedCoordinate* pxArticulation, Link *parent, 
        physx::PxTransform transform, LinkBody *body);
};
