#pragma once

#include "PxPhysicsAPI.h"
#include "ArticulationElements.h"
#include "IDisposable.h"

#include <unordered_map>
#include <string>

class Articulation :public IDisposable
{
// API BEGIN
public:
	std::unordered_map<std::string, Link*> linkMap;
	std::unordered_map<std::string, Joint*> jointMap;
public:
    int GetNDof() const;
    int GetNActiveJoints() const;
    void SetFixBaseFlag(bool shouldFixBase);
    const std::vector<int>& GetJointDofsInIdOrder() const;
public:
    std::vector<float> kps, kds, forceLimits;
    void SetKPs(const float kps[]);
    void SetKDs(const float kds[]);
    void SetForceLimits(const float forceLimits[]);
    void AddSPDForces(const float targetPositions[], float timeStep); // WXYZ or angle
// API END
private:
    std::vector<Joint*> jointList;
    std::vector<int> jointDofs;
    std::vector<std::string> jointNames;
private:
    physx::PxArticulationCache* mainCache;
    physx::PxArticulationCache* massMatrixCache;
    physx::PxArticulationCache* coriolisCache;
    physx::PxArticulationCache* gravityCache;
    physx::PxArticulationCache* externalForceCache;
public:
    physx::PxArticulationReducedCoordinate* pxArticulation;
public:
    Link* AddLink(std::string name, Link *parent, physx::PxTransform transform, LinkBody *body);
	Joint* AddSpericalJoint(std::string name, Link *link,
		physx::PxTransform parentPose, physx::PxTransform childPose);
	Joint* AddRevoluteJoint(std::string name, Link *link, physx::PxArticulationAxis::Enum axis,
		physx::PxTransform parentPose, physx::PxTransform childPose);
	Joint* AddFixedJoint(std::string name, Link *link, 
		physx::PxTransform parentPose, physx::PxTransform childPose);
public:
    void InitControl();
    void Dispose() override;
private:
    void AssignIndices();
};
