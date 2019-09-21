#pragma once

#include "PxPhysicsAPI.h"

inline physx::PxQuat ConvertTwistSwingToQuaternion(float t, float s1, float s2)
{
    physx::PxVec3 s(0, s1, s2);
    physx::PxQuat swingQuat = s.isZero() ? physx::PxQuat(0, 0, 0, 1) : physx::PxQuat(s.magnitude(), s.getNormalized());
    return swingQuat * physx::PxQuat(t, physx::PxVec3(1, 0, 0));
}

inline void SeparateTwistSwing(const physx::PxQuat& q, physx::PxQuat& swing, physx::PxQuat& twist)
{
	twist = q.x != 0.0f ? physx::PxQuat(q.x, 0, 0, q.w).getNormalized() : physx::PxQuat(physx::PxIdentity);
	swing = q * twist.getConjugate();
}
