#pragma once

#include "PxPhysicsAPI.h"

using namespace physx;

void loader();
void initControl();
void control(PxReal dt, int contactFlag);
void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask);

