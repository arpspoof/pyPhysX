#pragma once

#include "PxPhysicsAPI.h"
#include "LinkBody.h"
#include "ArticulationDescriptionNode.h"
#include "PrimitiveObjects.h"

#include <unordered_set>
#include <unordered_map>
#include <string>

class ArticulationTree {
	std::unordered_set<LinkBody*> linkBodies;
	std::unordered_map<std::string, ArticulationDescriptionNode*> nodeMap;
	ArticulationDescriptionNode *root;
public:
	LinkBody* CreateNullLinkBody();
	LinkBody* CreateSphereLinkBody(float mass, float radius, Material *material);
	LinkBody* CreateCapsuleLinkBody(float mass, float radius, float length, Material *material);
	LinkBody* CreateBoxLinkBody(float mass, float lenX, float lenY, float lenZ, Material *material);

	ArticulationDescriptionNode* CreateNULLDescriptionNode(std::string linkName, LinkBody *body);
	ArticulationDescriptionNode* CreateFixedDescriptionNode(
		std::string linkName, std::string jointName, LinkBody *body,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
	ArticulationDescriptionNode* CreateSpericalDescriptionNode(
		std::string linkName, std::string jointName, LinkBody *body,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);
	ArticulationDescriptionNode* CreateRevoluteDescriptionNode(
		std::string linkName, std::string jointName, LinkBody *body,
		physx::PxArticulationAxis::Enum axis,
		physx::PxVec3 posOffsetLinkToInboundJoint, physx::PxVec3 posOffsetJointToParentJoint);

	void SetRoot(std::string linkName);
	void Connect(std::string parentLinkName, std::string childLinkName);

	ArticulationDescriptionNode* GetRootNode() const;

	virtual ~ArticulationTree();
};
