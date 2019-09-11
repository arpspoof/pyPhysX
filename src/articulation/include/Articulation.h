#pragma once

#include "PxPhysicsAPI.h"
#include "ArticulationElements.h"
#include "IDisposable.h"

#include <unordered_map>
#include <string>

class Articulation :public IDisposable
{
public:
	std::unordered_map<std::string, Link*> linkMap;
	std::unordered_map<std::string, Joint*> jointMap;
public:
    Link* AddLink(std::string name, Link *parent, physx::PxTransform transform, LinkBody *body);
	Joint* AddSpericalJoint(std::string name, Link *link,
		physx::PxTransform parentPose, physx::PxTransform childPose);
	Joint* AddRevoluteJoint(std::string name, Link *link, physx::PxArticulationAxis::Enum axis,
		physx::PxTransform parentPose, physx::PxTransform childPose);
	Joint* AddFixedJoint(std::string name, Link *link, 
		physx::PxTransform parentPose, physx::PxTransform childPose);
public:
    Articulation();
    ~Articulation();
    void InitControl();
    void Dispose() override;
    physx::PxArticulationReducedCoordinate* GetPxArticulation() const;
private:
    void AssignIndices();
public:
    int GetNDof() const;
    int GetNActiveJoints() const;
    const int* GetJointDofsInIdOrder() const;
    void SetFixBaseFlag(bool shouldFixBase);
public:
    std::vector<float> kps, kds, forceLimits;
    void SetKPs(const float kps[]);
    void SetKDs(const float kds[]);
    void SetForceLimits(const float forceLimits[]);
    void AddSPDForces(const float targetPositions[], float timeStep); // WXYZ
private:
    std::vector<Joint*> jointList;
    std::vector<int> jointDofs;
    std::vector<std::string> jointNames;
private:
    physx::PxArticulationReducedCoordinate* pxArticulation;
    physx::PxArticulationCache* mainCache;
    physx::PxArticulationCache* massMatrixCache;
    physx::PxArticulationCache* coriolisCache;
    physx::PxArticulationCache* gravityCache;
    physx::PxArticulationCache* externalForceCache;
};
