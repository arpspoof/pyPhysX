#pragma once

#include "PxPhysicsAPI.h"
#include "IDisposable.h"
#include "Articulation.h"
#include "PrimitiveObjects.h"
#include "ArticulationTree.h"
#include "UrdfLoader.h"
#include "MathInterface.h"
#include "Actor.h"

#include <vector>
#include <unordered_set>

// API BEGIN
struct SceneDescription
{
    float gravity;
    int nWorkerThreads;
    bool enableGPUDynamics;
    bool enableGPUBroadPhase;

    SceneDescription();
};
// API END

class Foundation;

class Scene : public physx::PxSimulationEventCallback, public IDisposable
{
// API BEGIN
public:
    float timeStep;
public:
    Material* CreateMaterial(float staticFriction, float dynamicFriction, float restitution);
    Plane* CreatePlane(Material* material, vec3 planeNormal, float distance);
    Articulation* CreateArticulation(UrdfLoader* urdfLoader, Material* material, vec3 basePosition);
    Articulation* CreateArticulation(std::string urdfFilePath, Material* material, vec3 basePosition);

    Scene();
    void Step();
    void Dispose() override;

    const std::vector<std::pair<int, int>>& GetAllContactPairs() const;
// API END
public:
    Scene(Foundation* foundation, SceneDescription description, float timeStep);
    physx::PxScene* GetPxScene() const;
private:
    std::unordered_set<Material*> materials;
    std::unordered_set<physx::PxActor*> actors;
    std::unordered_set<Articulation*> articulations;
	void BuildArticulation(Articulation &ar, ArticulationDescriptionNode* startNode,
		Link* parentLink, physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
    Articulation* CreateArticulation(const ArticulationTree* tree, vec3 basePosition);
private:
    const Foundation* foundation;
    physx::PxScene* pxScene;
    physx::PxDefaultCpuDispatcher* pxCpuDispatcher;
private:
    std::vector<std::pair<int, int>> contacts;
    void onContact(const physx::PxContactPairHeader &pairHeader, 
		const physx::PxContactPair *pairs, physx::PxU32 nbPairs) override;
	void onConstraintBreak(physx::PxConstraintInfo * /*constraints*/, 
        physx::PxU32 /*count*/) override {}
	void onWake(physx::PxActor ** /*actors*/, physx::PxU32 /*count*/) override {}
	void onSleep(physx::PxActor ** /*actors*/, physx::PxU32 /*count*/) override {}
	void onTrigger(physx::PxTriggerPair * /*pairs*/, physx::PxU32 /*count*/) override {}
	void onAdvance(const physx::PxRigidBody *const * /*bodyBuffer*/, 
		const physx::PxTransform * /*poseBuffer*/, const physx::PxU32 /*count*/) override {}
public:
    static physx::PxFilterFlags CollisionShader(
        physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
        physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
        physx::PxPairFlags& pairFlags, const void* /*constantBlock*/, physx::PxU32 /*constantBlockSize*/);
};
