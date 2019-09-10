#pragma once

#include "PxPhysicsAPI.h"
#include "IDisposable.h"

class Foundation :public IDisposable
{
private:
    Foundation();
    ~Foundation();
public:
    void Dispose() override;
    physx::PxPhysics* GetPxPhysics() const;
    physx::PxCudaContextManager* GetPxCudaContextManager() const;
private:
    physx::PxFoundation*            pxFoundation;
    physx::PxPhysics*               pxPhysics;
    physx::PxDefaultAllocator		pxAllocator;
    physx::PxDefaultErrorCallback	pxErrorCallback;
    physx::PxCudaContextManager*    pxCudaContextManager;
private:
    static Foundation foundationGlobal;
public:
    static const Foundation* GetFoundation();
};
