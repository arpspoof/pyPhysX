#pragma once

#include "PxPhysicsAPI.h"

class vec3
{
public:
    float x, y, z;
    vec3() :x(0), y(0), z(0) {}
    vec3(float x, float y, float z) :x(x), y(y), z(z) {}
    operator physx::PxVec3() const { return physx::PxVec3(x, y, z); }
};
