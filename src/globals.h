#pragma once

#include "PxPhysicsAPI.h"
#include "articulationTree.h"

using namespace physx;

extern PxDefaultAllocator		gAllocator;
extern PxDefaultErrorCallback	gErrorCallback;

extern PxFoundation*			gFoundation;
extern PxPhysics*				gPhysics;

extern PxDefaultCpuDispatcher*	gDispatcher;
extern PxScene*					gScene;

extern PxMaterial*				gMaterial;

extern PxPvd*					gPvd;

extern PxArticulationReducedCoordinate*			gArticulation;
extern PxArticulationJointReducedCoordinate*	gDriveJoint;
extern PxArticulationCache*						gCache;

void loader();
void initControl();
void control(PxReal dt, int contactFlag);
void keyHandler(unsigned char key, const PxTransform& /*camera*/);
void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask);

enum CollisionGroup {
	Ground = 1 << 0,
	LeftFoot = 1 << 1,
	RightFoot = 1 << 2
};
