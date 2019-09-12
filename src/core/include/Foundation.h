#pragma once

#include "PxPhysicsAPI.h"
#include "IDisposable.h"
#include "Scene.h"

#include <unordered_set>

class Foundation :public IDisposable
{
// API BEGIN
public:
    Foundation();
    void Dispose() override;
    Scene* CreateScene(SceneDescription description, float timeStep);
// API END
public:
    physx::PxPhysics* GetPxPhysics() const;
    physx::PxCudaContextManager* GetPxCudaContextManager() const;
private:
    physx::PxFoundation*            pxFoundation;
    physx::PxPhysics*               pxPhysics;
    physx::PxDefaultAllocator		pxAllocator;
    physx::PxDefaultErrorCallback	pxErrorCallback;
    physx::PxCudaContextManager*    pxCudaContextManager;
    std::unordered_set<Scene*> scenes;
};
