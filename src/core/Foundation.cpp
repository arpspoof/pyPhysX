#include "Foundation.h"

using namespace physx;

Foundation::Foundation()
{
    pxFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, pxAllocator, pxErrorCallback);
    pxPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *pxFoundation, PxTolerancesScale(), true, nullptr);
    
    PxInitExtensions(*pxPhysics, nullptr);

    PxCudaContextManagerDesc cudaContextManagerDesc;
    pxCudaContextManager = PxCreateCudaContextManager(*pxFoundation, cudaContextManagerDesc, PxGetProfilerCallback());
}

Foundation::~Foundation()
{
    dispose();
}

void Foundation::dispose()
{
    pxPhysics->release();
    PxCloseExtensions();
    pxFoundation->release();
}

Foundation Foundation::foundationGlobal;

const Foundation* Foundation::GetFoundation()
{
    return &foundationGlobal;
}

PxPhysics* Foundation::GetPxPhysics() const 
{
    return pxPhysics;
}

PxCudaContextManager* Foundation::GetPxCudaContextManager() const
{
    return pxCudaContextManager;
}
