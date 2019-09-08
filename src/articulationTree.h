#pragma once

#include <PxPhysicsAPI.h>
#include <vector>
#include <unordered_map>
#include <cassert>

#include "linkBody.h"

class Link;

class Joint {
public:
	Link *parentLink;
	Link *childLink;
	int nDof;
	int cacheIndex;
	physx::PxArticulationJointReducedCoordinate *joint;
	physx::PxVec3 globalPositionOffset;
	virtual void enableDrive(std::string name) = 0;
protected:
	Joint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
};

class FixedJoint : public Joint {
public:
	FixedJoint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
	void enableDrive(std::string name) override;
};

class SphericalJoint : public Joint {
public:
	SphericalJoint(Link *link, physx::PxTransform parentPose, physx::PxTransform childPose);
	void enableDrive(std::string name) override;
};

class RevoluteJoint : public Joint {
public:
	physx::PxArticulationAxis::Enum axis;
	RevoluteJoint(Link *link, physx::PxArticulationAxis::Enum axis,
		physx::PxTransform parentPose, physx::PxTransform childPose);
	void enableDrive(std::string name) override;
};

class Link {
public:
	Link *parentLink;
	std::vector<Link*> childLinks;
	physx::PxArticulationLink *link;
	Joint *inboundJoint;
	physx::PxVec3 globalPositionOffset;
	Link(Link *parent, physx::PxTransform transform, LinkBody *body);
};

class Articulation {
public:
	std::unordered_map<std::string, Link*> linkMap;
	std::unordered_map<std::string, Joint*> jointMap;

	Link* addLink(std::string name, Link *parent, physx::PxTransform transform, LinkBody *body) {
		Link *link = new Link(parent, transform, body);
		linkMap[name] = link;
		return link;
	}

	Joint* addSpericalJoint(std::string name, Link *link,
		physx::PxTransform parentPose, physx::PxTransform childPose) {
		Joint *joint = new SphericalJoint(link, parentPose, childPose);
		jointMap[name] = joint;
		return joint;
	}

	Joint* addRevoluteJoint(std::string name, Link *link, physx::PxArticulationAxis::Enum axis,
		physx::PxTransform parentPose, physx::PxTransform childPose) {
		Joint *joint = new RevoluteJoint(link, axis, parentPose, childPose);
		jointMap[name] = joint;
		return joint;
	}

	Joint* addFixedJoint(std::string name, Link *link, 
		physx::PxTransform parentPose, physx::PxTransform childPose) {
		Joint *joint = new FixedJoint(link, parentPose, childPose);
		jointMap[name] = joint;
		return joint;
	}

	virtual ~Articulation() {
		for (auto& it : linkMap) {
			delete it.second;
		}
		for (auto& it : jointMap) {
			delete it.second;
		}
	}
};

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
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint)
		:linkName(linkName), jointName(jointName), body(body),
		posOffsetJointToParentJoint(posOffsetJointToParentJoint),
		posOffsetLinkToInboundJoint(posOffsetLinkToInboundJoint) {}
	virtual Joint* createJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const = 0;
public:
	Link* createLink(Articulation& ar, Link *parentLink,
		physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
};

class NULLDescriptionNode : public ArticulationDescriptionNode {
public:
	NULLDescriptionNode(std::string linkName, NULLLinkBody *body)
		: ArticulationDescriptionNode(linkName, "", body,
			physx::PxVec3(0, 0, 0), physx::PxVec3(0, 0, 0)) {}
	Joint* createJoint(Articulation&, Link *,
		physx::PxTransform,
		physx::PxTransform) const override {
		return NULL;
	}
};

class FixedDescriptionNode : public ArticulationDescriptionNode {
public:
	FixedDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint)
		: ArticulationDescriptionNode(linkName, jointName, body,
			posOffsetLinkToInboundJoint, posOffsetJointToParentJoint) {}
	Joint* createJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const override;
};

class SpericalDescriptionNode : public ArticulationDescriptionNode {
public:
	SpericalDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint)
		: ArticulationDescriptionNode(linkName, jointName, body,
			posOffsetLinkToInboundJoint, posOffsetJointToParentJoint) {}
	Joint* createJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const override;
};

class RevoluteDescriptionNode : public ArticulationDescriptionNode {
	physx::PxArticulationAxis::Enum axis;
public:
	RevoluteDescriptionNode(std::string linkName, std::string jointName, LinkBody *body,
		physx::PxArticulationAxis::Enum axis,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint)
		: ArticulationDescriptionNode(linkName, jointName, body,
			posOffsetLinkToInboundJoint, posOffsetJointToParentJoint), axis(axis) {}
	Joint* createJoint(Articulation& ar, Link *link,
		physx::PxTransform parentPose,
		physx::PxTransform childPose) const override;
};

class ArticulationTree {
	ArticulationDescriptionNode *root;
	std::unordered_map<std::string, ArticulationDescriptionNode*> nodeMap;
public:
	void addNULLDescriptionNode(const NULLDescriptionNode &node);
	void addFixedDescriptionNode(const FixedDescriptionNode &node);
	void addSpericalDescriptionNode(const SpericalDescriptionNode &node);
	void addRevoluteDescriptionNode(const RevoluteDescriptionNode &node);
	void setRoot(std::string linkName);
	void connect(std::string parentLinkName, std::string childLinkName);
	void buildArticulation(Articulation &ar);

	virtual ~ArticulationTree();
private:
	void buildArticulation(Articulation &ar, ArticulationDescriptionNode *startNode,
		Link *parentLink,
		physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
};
