#pragma once

#include "Actor.h"

class Material
{
public:
    physx::PxMaterial* pxMaterial;
};

class Plane :public RigidActorStatic
{
};
