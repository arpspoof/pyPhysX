#pragma once

#include "PxPhysicsAPI.h"

class Actor
{
public:
    physx::PxActor* pxActor;
};

class RigidActor :public Actor
{
public:
    void setupCollisionFiltering(int collisionGroup, int collisionMask);
};

class RigidActorStatic :public RigidActor
{
};
