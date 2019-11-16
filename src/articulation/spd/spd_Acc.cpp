#include "Articulation.h"
#include "CommonMath.h"
#include "sparseLTL.h"
#include "Eigen/Dense"

#include <cassert>

using namespace std;
using namespace physx;
using namespace Eigen;

extern PxQuat g_JointQuat[256];
extern bool debug;

void Articulation::AddSPDForcesAcc(const std::vector<float>& targetPositions, float timeStep)
{
    extern const float* g_InvD_Root_Kd;
    g_InvD_Root_Kd = nullptr;

    int nDof = GetNDof();

    pxArticulation->commonInit();
    pxArticulation->computeGeneralizedMassMatrix(*massMatrixCache, false);

    pxArticulation->copyInternalStateToCache(*coriolisCache, PxArticulationCache::eVELOCITY);
    pxArticulation->computeCoriolisAndCentrifugalForce(*coriolisCache, true);

    pxArticulation->computeGeneralizedGravityForce(*gravityCache, true);
    pxArticulation->computeGeneralizedExternalForce(*externalForceCache, true);

    VectorXf centrifugalCoriolisGravityExternal(nDof + 6);
    for (int i = 0; i < nDof + 6; i++) {
        centrifugalCoriolisGravityExternal((i + 6) % (nDof + 6)) = (
            coriolisCache->jointForce[i] + 
            gravityCache->jointForce[i] + 
            externalForceCache->jointForce[i]
        );
    }

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    vector<float> proportionalAccPlusQDotDeltaT(nDof + 6);
    vector<float> derivativeAcc(nDof + 6);

    int nJoints = GetNJoints();
    int targetPositionsIndex = 7;

    for (int i = 0; i < nJoints; i++) {
        const Joint* joint = jointList[i];

        const int jointDof = joint->nDof;
        const int cacheIndex = joint->cacheIndex;

        if (jointDof == 3) {
            PxQuat targetPosition(
                targetPositions[targetPositionsIndex + 1],
                targetPositions[targetPositionsIndex + 2],
                targetPositions[targetPositionsIndex + 3],
                targetPositions[targetPositionsIndex]
            );
            UniformQuaternion(targetPosition); // Never trust user input

            PxVec3 kp(
                kps[cacheIndex],
                kps[cacheIndex + 1],
                kps[cacheIndex + 2]
            );

            targetPosition = frameTransform.getConjugate() * targetPosition * frameTransform;

            if (targetPosition.w < 0) {
                targetPosition = -targetPosition;
            }

            PxQuat localRotation = g_JointQuat[cacheIndex];

            PxQuat posDifference = targetPosition * localRotation.getConjugate();
            UniformQuaternion(posDifference);

            PxVec3 axis;
            PxReal angle;
            posDifference.toRadiansAndUnitAxis(angle, axis);
            axis *= angle;

            PxVec3 proportionalForceInParentFrame(
                kps[cacheIndex] * axis.x,
                kps[cacheIndex + 1] * axis.y,
                kps[cacheIndex + 2] * axis.z
            );
            PxVec3 proportionalForceInChildFrame = localRotation.getConjugate().rotate(proportionalForceInParentFrame);

            proportionalAccPlusQDotDeltaT[cacheIndex + 6] = proportionalForceInChildFrame[0] - 
                timeStep * velocities[cacheIndex] * kps[cacheIndex];
            proportionalAccPlusQDotDeltaT[cacheIndex + 7] = proportionalForceInChildFrame[1] -
                timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
            proportionalAccPlusQDotDeltaT[cacheIndex + 8] = proportionalForceInChildFrame[2] -
                timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

            derivativeAcc[cacheIndex + 6] = -kds[cacheIndex] * velocities[cacheIndex];
            derivativeAcc[cacheIndex + 7] = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
            derivativeAcc[cacheIndex + 8] = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

            targetPositionsIndex += 4;
        }
        else if (jointDof == 1) {
            proportionalAccPlusQDotDeltaT[cacheIndex + 6] = kps[cacheIndex] * (
                targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
                timeStep * velocities[cacheIndex]
            );
            derivativeAcc[cacheIndex + 6] = -kds[cacheIndex] * velocities[cacheIndex];

            targetPositionsIndex += 1;
        }
        else if (jointDof == 0) {
            continue;
        }
        else {
            printf("no controller defined for Dof %d\n", jointDof);
            assert(false);
        }
    }

    VectorXf qdd(nDof);
    for (int i = 0; i < nDof; i++) {
        qdd(i) = (proportionalAccPlusQDotDeltaT[i + 6] + derivativeAcc[i + 6]) / (1 + kds[i] * timeStep);
    }
    
    VectorXf a0(6);

    PxVec3 rootGlobalPosition = rootLink->link->getGlobalPose().p;
    PxQuat rootGlobalRotation = rootLink->link->getGlobalPose().q;

    PxVec3 rootGlobalLinearVelocity = rootLink->link->getLinearVelocity();
    PxVec3 rootGlobalAngularVelocity = rootLink->link->getAngularVelocity();
    
    PxVec3 rootGlobalProportionalLinearForcePlusQDotDeltaT(
        root_kps[0] * (targetPositions[0] - rootGlobalPosition[0] - timeStep * rootGlobalLinearVelocity[0]),
        root_kps[1] * (targetPositions[1] - rootGlobalPosition[1] - timeStep * rootGlobalLinearVelocity[1]),
        root_kps[2] * (targetPositions[2] - rootGlobalPosition[2] - timeStep * rootGlobalLinearVelocity[2])
    );
    PxVec3 rootGlobalDerivativeLinearForce(
        -root_kds[0] * rootGlobalLinearVelocity[0],
        -root_kds[1] * rootGlobalLinearVelocity[1],
        -root_kds[2] * rootGlobalLinearVelocity[2]
    );

    PxVec3 rootLocalProportionalLinearForcePlusQDotDeltaT = 
        rootGlobalRotation.rotateInv(rootGlobalProportionalLinearForcePlusQDotDeltaT);
    PxVec3 rootLocalDerivativeLinearForce = rootGlobalRotation.rotateInv(rootGlobalDerivativeLinearForce);

    PxQuat rootGlobalTargetRotationUser(targetPositions[4], targetPositions[5], targetPositions[6], targetPositions[3]);
    rootGlobalTargetRotationUser.normalize();
    
    // transform from { x:front, y:up, z:right } to { x:up, y:back, z:right }
    PxQuat rootGlobalTargetRotation = rootGlobalTargetRotationUser * frameTransform;

    PxQuat diffQuat = rootGlobalTargetRotation * rootGlobalRotation.getConjugate();
    if (PxAbs(diffQuat.w) < 0.70710678118f) {
        diffQuat = (-rootGlobalTargetRotation) * rootGlobalRotation.getConjugate();
    }
    UniformQuaternion(diffQuat);

    PxVec3 diffRotExpMapGlobal = QuatToExpMap(diffQuat);
    PxVec3 rootGlobalProportionalAccPlusQDotDeltaT(
        root_kps[3] * (diffRotExpMapGlobal[0] - timeStep * rootGlobalAngularVelocity[0]),
        root_kps[4] * (diffRotExpMapGlobal[1] - timeStep * rootGlobalAngularVelocity[1]),
        root_kps[5] * (diffRotExpMapGlobal[2] - timeStep * rootGlobalAngularVelocity[2])
    );
    PxVec3 rootGlobalDerivativeAcc(
        -root_kds[3] * rootGlobalAngularVelocity[0],
        -root_kds[4] * rootGlobalAngularVelocity[1],
        -root_kds[5] * rootGlobalAngularVelocity[2]
    );

    PxVec3 rootLocalproportionalAccPlusQDotDeltaT = 
        rootGlobalRotation.rotateInv(rootGlobalProportionalAccPlusQDotDeltaT);
    PxVec3 rootLocalderivativeAcc = rootGlobalRotation.rotateInv(rootGlobalDerivativeAcc);

    for (int i = 0; i < 3; i++) {
        proportionalAccPlusQDotDeltaT[i] = rootLocalProportionalLinearForcePlusQDotDeltaT[i];
        derivativeAcc[i] = rootLocalDerivativeLinearForce[i];
        a0(i) = (proportionalAccPlusQDotDeltaT[i] + derivativeAcc[i]) / (1 + root_kds[i] * timeStep);
    }
    for (int i = 3; i < 6; i++) {
        proportionalAccPlusQDotDeltaT[i] = rootLocalproportionalAccPlusQDotDeltaT[i - 3];
        derivativeAcc[i] = rootLocalderivativeAcc[i - 3];
        a0(i) = (proportionalAccPlusQDotDeltaT[i] + derivativeAcc[i]) / (1 + root_kds[i] * timeStep);
    }

    MatrixXf massMatrix(nDof + 6, nDof + 6);
    for (int i = 0; i < nDof + 6; i++) {
        for (int j = i; j < nDof + 6; j++) {
            massMatrix(i, j) = massMatrix(j, i) = massMatrixCache->massMatrix[i * (nDof + 6) + j];
        }
    }

    VectorXf acc(nDof + 6);
    for (int i = 0; i < 6; i++) acc(i) = a0(i);
    for (int i = 0; i < nDof; i++) acc(i + 6) = qdd(i);

    VectorXf p0c = centrifugalCoriolisGravityExternal.head(6);
    MatrixXf F = massMatrix.block<6, 34>(0, 0); // hard coded 28
    VectorXf obj = F * acc + p0c;

    if (debug) {
        printf("acc is:\n");
        for (int i = 0; i < 6 + nDof; i++) printf("%f, ", acc(i));
        printf("\n");
    }

    for (int i = 0; i < 10000; i++) {
        if (debug) {
         /*   printf("objective is:\n");
            for (int i = 0; i < 6; i++) printf("%f, ", obj(i));
            printf("\n");
            printf("acc is:\n");
            for (int i = 0; i < 6 + nDof; i++) printf("%f, ", acc(i));
            printf("\n");*/
        }
        if (obj.norm() < 0.1f) break;
        float alpha = 0.0001f;
        acc -= alpha * F.transpose() * obj;
        obj = F * acc + p0c;
    }

    VectorXf forceFromAcc = massMatrix * acc + centrifugalCoriolisGravityExternal;
    for (int i = 0; i < nDof; i++) {
        forces[i] = forceFromAcc(i + 6);
    }

    if (debug) {
        printf("accelerations are:\n");
        for (int i = 0; i < nDof + 6; i++) printf("%f, ", acc(i));
        printf("\n");
        printf("forces are:\n");
        for (int i = 0; i < nDof + 6; i++) printf("%f, ", forceFromAcc(i));
        printf("\n");
    }

/*    extern float g_RootExternalSpatialForce[6];
    for (int i = 0; i < 6; i++) {
        g_RootExternalSpatialForce[i] = forceFromAcc(i);
    }*/

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);
}
