#include "Articulation.h"
#include "CommonMath.h"

#include <cassert>

using namespace std;
using namespace physx;

vector<float> Articulation::GetJointPositionsQuaternion() const
{
    vector<float> result(7 + 4*nSphericalJoint + nRevoluteJoint);

    PxTransform rootPose = rootLink->link->getGlobalPose();
    PxQuat rootRotation = rootPose.q * frameTransform.getConjugate();
    UniformQuaternion(rootRotation);
    
    result[0] = rootPose.p.x;
    result[1] = rootPose.p.y;
    result[2] = rootPose.p.z;

    result[3] = rootRotation.w;
    result[4] = rootRotation.x;
    result[5] = rootRotation.y;
    result[6] = rootRotation.z;

    int nJoints = GetNJoints();
    int resultIndex = 7;

    for (int i = 0; i < nJoints; i++) {
        const int jointDof = jointList[i]->nDof;
        const int cacheIndex = jointList[i]->cacheIndex;

        if (jointDof == 1) {
            result[resultIndex++] = mainCache->jointPosition[cacheIndex];
        }
        else if (jointDof == 3) {
            PxQuat rotation = frameTransform * ConvertTwistSwingToQuaternion(
                mainCache->jointPosition[cacheIndex],
                mainCache->jointPosition[cacheIndex + 1],
                mainCache->jointPosition[cacheIndex + 2]
            ) * frameTransform.getConjugate();
            UniformQuaternion(rotation);
            
            result[resultIndex] = rotation.w;
            result[resultIndex + 1] = rotation.x;
            result[resultIndex + 2] = rotation.y;
            result[resultIndex + 3] = rotation.z;

            resultIndex += 4;
        }
    }

    return result;
}

void Articulation::SetJointPositionsQuaternion(const vector<float>& positions) const
{
    assert((int)positions.size() == 7 + 4*nSphericalJoint + nRevoluteJoint);

    PxVec3 rootGlobalTranslation(positions[0], positions[1], positions[2]);
    PxQuat rootGlobalRotation(positions[4], positions[5], positions[6], positions[3]);
    UniformQuaternion(rootGlobalRotation); 
    
    // transform from { x:front, y:up, z:right } to { x:up, y:back, z:right }
    PxQuat rootPose = rootGlobalRotation * frameTransform;
    UniformQuaternion(rootPose);

    pxArticulation->teleportRootLink(PxTransform(rootGlobalTranslation, rootPose), true);

    int nJoints = GetNJoints();
    int inputIndex = 7;

    for (int i = 0; i < nJoints; i++) {
        const int jointDof = jointList[i]->nDof;
        const int cacheIndex = jointList[i]->cacheIndex;

        if (jointDof == 1) {
            mainCache->jointPosition[cacheIndex] = positions[inputIndex++];
        }
        else if (jointDof == 3) {
            PxQuat rotation(
                positions[inputIndex + 1], positions[inputIndex + 2],
                positions[inputIndex + 3], positions[inputIndex]
            );

            UniformQuaternion(rotation); // Never trust user input
            rotation = frameTransform.getConjugate() * rotation * frameTransform;
            
            PxQuat twist, swing;
            SeparateTwistSwing(rotation, swing, twist);
            float theta0 = PxAtan2(twist.x, (1.f + twist.w)) * 4.f;
            float theta1 = PxAtan2(swing.y, (1.f + swing.w)) * 4.f;
            float theta2 = PxAtan2(swing.z, (1.f + swing.w)) * 4.f;
            
            mainCache->jointPosition[cacheIndex] = theta0;
            mainCache->jointPosition[cacheIndex + 1] = theta1;
            mainCache->jointPosition[cacheIndex + 2] = theta2;

            inputIndex += 4;
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::ePOSITION);
}

vector<float> Articulation::GetJointVelocitiesPack4() const
{
    vector<float> result(7 + 4*nSphericalJoint + nRevoluteJoint);

    PxVec3 rootLinearVelocity = rootLink->link->getLinearVelocity();
    result[0] = rootLinearVelocity.x;
    result[1] = rootLinearVelocity.y;
    result[2] = rootLinearVelocity.z;

    PxVec3 rootAngularVelocity = rootLink->link->getAngularVelocity();
    result[3] = rootAngularVelocity.x;
    result[4] = rootAngularVelocity.y;
    result[5] = rootAngularVelocity.z;
    result[6] = 0; // pack4

    int nJoints = GetNJoints();
    int resultIndex = 7;

    for (int i = 0; i < nJoints; i++) {
        const int jointDof = jointList[i]->nDof;
        const int cacheIndex = jointList[i]->cacheIndex;

        if (jointDof == 1) {
            result[resultIndex++] = mainCache->jointVelocity[cacheIndex];
        }
        else if (jointDof == 3) {
            PxVec3 angularV(mainCache->jointVelocity[cacheIndex],
                mainCache->jointVelocity[cacheIndex + 1], mainCache->jointVelocity[cacheIndex + 2]);
            angularV = frameTransform.rotate(angularV);
            result[resultIndex] = angularV.x;
            result[resultIndex + 1] = angularV.y;
            result[resultIndex + 2] = angularV.z;
            result[resultIndex + 3] = 0; // pack4

            resultIndex += 4;
        }
    }

    return result;
}

void Articulation::SetJointVelocitiesPack4(const std::vector<float>& velocities) const
{
    assert((int)velocities.size() == 7 + 4*nSphericalJoint + nRevoluteJoint);

    PxVec3 rootLinearVelocity(velocities[0], velocities[1], velocities[2]);
    rootLink->link->setLinearVelocity(rootLinearVelocity);

    PxVec3 rootAngularVelocity(velocities[3], velocities[4], velocities[5]);
    rootLink->link->setAngularVelocity(rootAngularVelocity);

    int nJoints = GetNJoints();
    int inputIndex = 7;

    for (int i = 0; i < nJoints; i++) {
        const int jointDof = jointList[i]->nDof;
        const int cacheIndex = jointList[i]->cacheIndex;

        if (jointDof == 1) {
            mainCache->jointVelocity[cacheIndex] = velocities[inputIndex++];
        }
        else if (jointDof == 3) {
            PxVec3 angularV(velocities[inputIndex], velocities[inputIndex + 1], velocities[inputIndex + 2]);
            angularV = frameTransform.rotateInv(angularV);

            mainCache->jointVelocity[cacheIndex] = angularV.x;
            mainCache->jointVelocity[cacheIndex + 1] = angularV.y;
            mainCache->jointVelocity[cacheIndex + 2] = angularV.z;

            inputIndex += 4;
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eVELOCITY);
}
