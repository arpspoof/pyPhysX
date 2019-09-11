#include "ArticulationTree.h"
#include <cassert>

using namespace physx;

void ArticulationTree::AddNULLDescriptionNode(const NULLDescriptionNode &node) {
	NULLDescriptionNode *copyNode = new NULLDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::AddFixedDescriptionNode(const FixedDescriptionNode &node) {
	FixedDescriptionNode *copyNode = new FixedDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::AddSpericalDescriptionNode(const SpericalDescriptionNode &node) {
	SpericalDescriptionNode *copyNode = new SpericalDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::AddRevoluteDescriptionNode(const RevoluteDescriptionNode &node) {
	RevoluteDescriptionNode *copyNode = new RevoluteDescriptionNode(node);
	nodeMap[node.linkName] = copyNode;
}

void ArticulationTree::SetRoot(std::string linkName) {
	assert(nodeMap.find(linkName) != nodeMap.end());
	root = nodeMap[linkName];
	root->parent = NULL;
}

void ArticulationTree::Connect(std::string parentLinkName, std::string childLinkName) {
	assert(nodeMap.find(parentLinkName) != nodeMap.end());
	assert(nodeMap.find(childLinkName) != nodeMap.end());
	ArticulationDescriptionNode *parent = nodeMap[parentLinkName];
	ArticulationDescriptionNode *child = nodeMap[childLinkName];
	child->parent = parent;
	parent->children.push_back(child);
}

void ArticulationTree::BuildArticulation(Articulation &ar, PxVec3 basePosition) {
	assert(root != NULL);
	BuildArticulation(ar, root, NULL, basePosition, basePosition);
}

void ArticulationTree::BuildArticulation(Articulation &ar, ArticulationDescriptionNode *startNode,
	Link *parentLink, PxVec3 parentJointPos, PxVec3 parentLinkPos) const {
	Link *link = startNode->CreateLink(ar, parentLink, parentJointPos, parentLinkPos);
	for (auto it : startNode->children) {
		BuildArticulation(ar, it, link,
			parentJointPos + startNode->posOffsetJointToParentJoint,
			link->globalPositionOffset);
	}
}

ArticulationTree::~ArticulationTree() {
	for (auto& it : nodeMap) {
		delete it.second;
	}
}
