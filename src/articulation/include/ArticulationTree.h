#pragma once

#include "PxPhysicsAPI.h"
#include "ArticulationDescriptionNode.h"

#include <unordered_map>

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
		Link *parentLink, physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
};
