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
    void SetFixBaseFlag(bool shouldFixBase);
private:
    physx::PxArticulationReducedCoordinate* pxArticulation;
    physx::PxArticulationCache* mainCache;
    physx::PxArticulationCache* massMatrixCache;
    physx::PxArticulationCache* coriolisCache;
    physx::PxArticulationCache* gravityCache;
    physx::PxArticulationCache* externalForceCache;
};
