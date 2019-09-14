#pragma once

#include "PxPhysicsAPI.h"

class vec3
{
public:
// API BEGIN
    float x, y, z;
    vec3() :x(0), y(0), z(0) {}
    vec3(float x, float y, float z) :x(x), y(y), z(z) {}
// API END
    operator physx::PxVec3() const { return physx::PxVec3(x, y, z); }
};

class quat
{
public:
// API BEGIN
    float x, y, z, w;
    quat() :x(0), y(0), z(0), w(1) {}
    quat(float x, float y, float z, float w) :x(x), y(y), z(z), w(w) {}
// API END
    operator physx::PxQuat() const { return physx::PxQuat(w, y, z ,w); }
};

class Transform
{
public:
// API BEGIN
    vec3 p;
    quat q;
    Transform() {}
    Transform(vec3 p, quat q) :p(p), q(q) {}
// API END
    operator physx::PxTransform() const { return physx::PxTransform(p, q); }
};
