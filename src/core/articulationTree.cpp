#include "articulationTree.h"
#include "globals.h"
#include "config.h"

using namespace physx;

static PxQuat rtz(PxPi / 2, PxVec3(0, 0, 1));
static PxQuat rtzinv(-PxPi / 2, PxVec3(0, 0, 1));

static PxTransform getJointPose(PxVec3 offset) {
	return PxTransform(rtzinv.rotate(offset));
}

Joint::Joint(Link *link, PxTransform parentPose, PxTransform childPose)
	: nDof(-1), cacheIndex(-1) {
	joint = static_cast<PxArticulationJointReducedCoordinate*>(link->link->getInboundJoint());
	joint->setParentPose(parentPose);
	joint->setChildPose(childPose);
}

FixedJoint::FixedJoint(Link *link, PxTransform parentPose, PxTransform childPose)
	: Joint(link, parentPose, childPose) {
	joint->setJointType(PxArticulationJointType::eFIX);
}

void FixedJoint::enableDrive(std::string name) {
}

SphericalJoint::SphericalJoint(Link *link, PxTransform parentPose, PxTransform childPose)
	: Joint(link, parentPose, childPose) {
	joint->setJointType(PxArticulationJointType::eSPHERICAL);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
	joint->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	joint->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);

	PxReal twistLimit = getConfigF("C_TWIST_LIMIT");
	joint->setLimit(PxArticulationAxis::eTWIST, -twistLimit, twistLimit);
}

void SphericalJoint::enableDrive(std::string name) {
	PxReal kp = getConfigF("P_KP_" + name);
	PxReal kd = getConfigF("P_KD_" + name);
	PxReal fl = getConfigF("P_FL_" + name);

	joint->setDrive(PxArticulationAxis::eTWIST, kp, kd, fl);
	joint->setDrive(PxArticulationAxis::eSWING1, kp, kd, fl);
	joint->setDrive(PxArticulationAxis::eSWING2, kp, kd, fl);
}

RevoluteJoint::RevoluteJoint(Link *link, PxArticulationAxis::Enum axis,
	PxTransform parentPose, PxTransform childPose)
	: Joint(link, parentPose, childPose), axis(axis) {
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(axis, PxArticulationMotion::eFREE);
}

void RevoluteJoint::enableDrive(std::string name) {
	PxReal kp = getConfigF("P_KP_" + name);
	PxReal kd = getConfigF("P_KD_" + name);
	PxReal fl = getConfigF("P_FL_" + name);

	joint->setDrive(axis, kp, kd, fl);
}

Link::Link(Link *parent, PxTransform transform, LinkBody *body)
	:parentLink(parent), inboundJoint(nullptr) {
	link = gArticulation->createLink(parent ? parent->link : NULL, transform);
	if (body->hasGeometry) {
		PxRigidActorExt::createExclusiveShape(*link, body->getGeometry(), *gMaterial);
	}
	if (parent) {
		parent->childLinks.push_back(this);
	}
	PxRigidBodyExt::updateMassAndInertia(*link, body->getDensity());
}

Link* ArticulationDescriptionNode::createLink(Articulation& ar, Link *parentLink,
	PxVec3 parentJointPos, PxVec3 parentLinkPos) const {
	PxVec3 jointPos = parentJointPos + posOffsetJointToParentJoint;
	PxVec3 linkPos = jointPos + posOffsetLinkToInboundJoint;
	Link *link = ar.addLink(linkName, parentLink, PxTransform(linkPos, rtz), body);
	link->globalPositionOffset = linkPos;
	if (parentLink) {
		Joint *joint = createJoint(ar, link,
			getJointPose(jointPos - parentLinkPos),
			getJointPose(jointPos - linkPos)
		);
		link->inboundJoint = joint;
		joint->globalPositionOffset = jointPos;
		joint->childLink = link;
		joint->parentLink = parentLink;
	//	joint->enableDrive(jointName);
	}
	return link;
}

Joint* FixedDescriptionNode::createJoint(Articulation& ar, Link *link,
	physx::PxTransform parentPose,
	physx::PxTransform childPose) const {
	return ar.addFixedJoint(jointName, link, parentPose, childPose);
}

Joint* SpericalDescriptionNode::createJoint(Articulation& ar, Link *link,
	physx::PxTransform parentPose,
	physx::PxTransform childPose) const {
	return ar.addSpericalJoint(jointName, link, parentPose, childPose);
}

Joint* RevoluteDescriptionNode::createJoint(Articulation& ar, Link *link,
	physx::PxTransform parentPose,
	physx::PxTransform childPose) const {
	return ar.addRevoluteJoint(jointName, link, axis, parentPose, childPose);
}

void ArticulationTree::addNULLDescriptionNode(const NULLDescriptionNode &node) {
	NULLDescriptionNode *copyNode = new NULLDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::addFixedDescriptionNode(const FixedDescriptionNode &node) {
	FixedDescriptionNode *copyNode = new FixedDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::addSpericalDescriptionNode(const SpericalDescriptionNode &node) {
	SpericalDescriptionNode *copyNode = new SpericalDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::addRevoluteDescriptionNode(const RevoluteDescriptionNode &node) {
	RevoluteDescriptionNode *copyNode = new RevoluteDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::setRoot(std::string linkName) {
	assert(nodeMap.find(linkName) != nodeMap.end());
	root = nodeMap[linkName];
	root->parent = NULL;
}

void ArticulationTree::connect(std::string parentLinkName, std::string childLinkName) {
	assert(nodeMap.find(parentLinkName) != nodeMap.end());
	assert(nodeMap.find(childLinkName) != nodeMap.end());
	ArticulationDescriptionNode *parent = nodeMap[parentLinkName];
	ArticulationDescriptionNode *child = nodeMap[childLinkName];
	child->parent = parent;
	parent->children.push_back(child);
}

void ArticulationTree::buildArticulation(Articulation &ar) {
	assert(root != NULL);
	PxVec3 basePos(0.f, getConfigF("T_BASE_HEIGHT"), 0.f);
	buildArticulation(ar, root, NULL, basePos, basePos);
}

void ArticulationTree::buildArticulation(Articulation &ar, ArticulationDescriptionNode *startNode,
	Link *parentLink, PxVec3 parentJointPos, PxVec3 parentLinkPos) const {
	Link *link = startNode->createLink(ar, parentLink, parentJointPos, parentLinkPos);
	for (auto it : startNode->children) {
		buildArticulation(ar, it, link,
			parentJointPos + startNode->posOffsetJointToParentJoint,
			link->globalPositionOffset);
	}
}

ArticulationTree::~ArticulationTree() {
	for (auto& it : nodeMap) {
		delete it.second;
	}
}
