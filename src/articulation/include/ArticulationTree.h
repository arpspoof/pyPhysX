#pragma once

#include "PxPhysicsAPI.h"
#include "ArticulationDescriptionNode.h"

#include <unordered_map>

class ArticulationTree {
	ArticulationDescriptionNode *root;
	std::unordered_map<std::string, ArticulationDescriptionNode*> nodeMap;
public:
	void AddNULLDescriptionNode(const NULLDescriptionNode &node);
	void AddFixedDescriptionNode(const FixedDescriptionNode &node);
	void AddSpericalDescriptionNode(const SpericalDescriptionNode &node);
	void AddRevoluteDescriptionNode(const RevoluteDescriptionNode &node);
	void SetRoot(std::string linkName);
	void Connect(std::string parentLinkName, std::string childLinkName);
	void BuildArticulation(Articulation &ar, physx::PxVec3 basePosition);

	virtual ~ArticulationTree();
private:
	void BuildArticulation(Articulation &ar, ArticulationDescriptionNode *startNode,
		Link *parentLink, physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
};
