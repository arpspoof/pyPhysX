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

	ArticulationDescriptionNode* GetRootNode() const;

	virtual ~ArticulationTree();
};
