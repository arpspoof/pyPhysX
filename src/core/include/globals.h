#pragma once

#include "PxPhysicsAPI.h"
#include "articulationTree.h"

using namespace physx;

extern PxMaterial*				gMaterial;

extern PxArticulationReducedCoordinate*			gArticulation;
extern PxArticulationCache*						gCache;

void loader();
void initControl();
void control(PxReal dt, int contactFlag);
void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask);

