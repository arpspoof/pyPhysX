#include "ArticulationTree.h"
#include "globals.h"
#include "config.h"

#include <cassert>

using namespace physx;

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
	Link *link = startNode->CreateLink(ar, parentLink, parentJointPos, parentLinkPos);
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
