#pragma once

#include "PxPhysicsAPI.h"
#include "IDisposable.h"

struct SceneDescription
{
    float gravity;
    int nWorkerThreads;
    bool enableGPUDynamics;
    bool enableGPUBroadPhase;

    SceneDescription();
};

class Scene;

class SceneObject
{
friend class Scene;
public:
    SceneObject(physx::PxActor* pxActor);
private:
    physx::PxActor* pxActor;
};

class Scene : public physx::PxSimulationEventCallback, IDisposable
{
public:
    Scene(SceneDescription description, float timeStep = 0.001f);
    ~Scene();
    void dispose() override;
public:
    void AddObject(SceneObject obj);
    void Step();
    physx::PxScene* GetPxScene() const;
private:
    void onContact(const physx::PxContactPairHeader &pairHeader, 
		const physx::PxContactPair *pairs, physx::PxU32 nbPairs) override;
	void onConstraintBreak(physx::PxConstraintInfo * /*constraints*/, 
        physx::PxU32 /*count*/) override {}
	void onWake(physx::PxActor ** /*actors*/, physx::PxU32 /*count*/) override {}
	void onSleep(physx::PxActor ** /*actors*/, physx::PxU32 /*count*/) override {}
	void onTrigger(physx::PxTriggerPair * /*pairs*/, physx::PxU32 /*count*/) override {}
	void onAdvance(const physx::PxRigidBody *const * /*bodyBuffer*/, 
		const physx::PxTransform * /*poseBuffer*/, const physx::PxU32 /*count*/) override {}
private:
    physx::PxScene* pxScene;
    physx::PxDefaultCpuDispatcher* pxCpuDispatcher;
public:
    float timeStep;
private:
    void reportContact(const physx::PxActor* actor0, const physx::PxActor* actor1);
};
