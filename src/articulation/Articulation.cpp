#include "Articulation.h"
#include "Foundation.h"
#include "CommonMath.h"
#include "sparseLTL.h"
#include "Eigen/Dense"

#include <algorithm>

using namespace physx;
using namespace std;
using namespace Eigen;

Link* Articulation::AddLink(std::string name, Link *parent, physx::PxTransform transform, LinkBody *body) 
{
    Link *link = new Link(pxArticulation, parent, transform, body);
    linkMap[name] = link;
    return link;
}

Joint* Articulation::AddSpericalJoint(std::string name, Link *link,
    physx::PxTransform parentPose, physx::PxTransform childPose) 
{
    Joint *joint = new SphericalJoint(link, parentPose, childPose);
    jointMap[name] = joint;
    return joint;
}

Joint* Articulation::AddRevoluteJoint(std::string name, Link *link, physx::PxArticulationAxis::Enum axis,
    physx::PxTransform parentPose, physx::PxTransform childPose) 
{
    Joint *joint = new RevoluteJoint(link, axis, parentPose, childPose);
    jointMap[name] = joint;
    return joint;
}

Joint* Articulation::AddFixedJoint(std::string name, Link *link, 
    physx::PxTransform parentPose, physx::PxTransform childPose) 
{
    Joint *joint = new FixedJoint(link, parentPose, childPose);
    jointMap[name] = joint;
    return joint;
}

void Articulation::Dispose() 
{
    pxArticulation->release();
    for (auto& it : linkMap) {
        delete it.second;
    }
    for (auto& it : jointMap) {
        delete it.second;
    }
}

void Articulation::InitControl(unordered_map<string, int>& jointIdMap)
{
    AssignIndices();

    for (auto &kvp : linkMap) {
        kvp.second->name = kvp.first;
    }
    for (auto &kvp : jointMap) {
        kvp.second->name = kvp.first;
    }

    mainCache = pxArticulation->createCache();
    massMatrixCache = pxArticulation->createCache();
    coriolisCache = pxArticulation->createCache();
    gravityCache = pxArticulation->createCache();
    externalForceCache = pxArticulation->createCache();

    int nDof = GetNDof();
    kps = vector<float>(nDof, 0);
    kds = vector<float>(nDof, 0);
    forceLimits = vector<float>(nDof, -1);

    typedef tuple<int, Joint*> PIJ;
    vector<PIJ> joints;
    
    for (auto &kvp : jointMap) {
        joints.push_back(make_tuple(jointIdMap[kvp.first], kvp.second));
    }

    sort(joints.begin(), joints.end(), [](PIJ &a, PIJ &b) { return get<0>(a) < get<0>(b); });

    for (PIJ &pijn : joints) {
        auto j = get<1>(pijn);
        j->id = get<0>(pijn);
        jointList.push_back(j);
        jointDofs.push_back(j->nDof);
    }

    parentIndexMapForSparseLTL = vector<int>(nDof + 6 + 1, -1);
    for (int k = 1; k <= 6; k++) parentIndexMapForSparseLTL[k] =  k - 1;

    for (auto &kvp : jointMap) {
        Joint* j = kvp.second;
        if (j->cacheIndex != -1) {
            Joint* p = j->parentLink->inboundJoint;
            while (p && p->cacheIndex == -1) p = p->parentLink->inboundJoint;
            
            if (p) parentIndexMapForSparseLTL[j->cacheIndex + 7] = p->cacheIndex + 6 + p->nDof;
            else parentIndexMapForSparseLTL[j->cacheIndex + 7] = 6;
            
            for (int k = 1; k < j->nDof; k++) {
                parentIndexMapForSparseLTL[j->cacheIndex + 7 + k] = j->cacheIndex + 7 + k - 1;
            }
        }
    }

    printf("parent index\n");
    for (int i = 0; i < (int)parentIndexMapForSparseLTL.size(); i++) {
        printf("%d ", parentIndexMapForSparseLTL[i]);
    }
    printf("\n");
}

void Articulation::AssignIndices() {
    typedef pair<PxU32, Link*> PIDL;
    vector<PIDL> linkIndices;
    for (auto &kvp : linkMap) {
        linkIndices.push_back(make_pair(kvp.second->link->getLinkIndex(), kvp.second));
    }
    sort(linkIndices.begin(), linkIndices.end(), [=](PIDL a, PIDL b) { return a.first < b.first; });

    nSphericalJoint = 0;
    nRevoluteJoint = 0;

    rootLink = nullptr;
    linkIdCacheIndexMap = vector<int>(pxArticulation->getNbLinks(), -1);

    int currentIndex = 0;

    for (PIDL &p : linkIndices) {
        int nDof = (int)p.second->link->getInboundJointDof();
        if (!p.second->inboundJoint) {
            rootLink = p.second;
            continue;
        }
        p.second->inboundJoint->nDof = nDof;
        p.second->inboundJoint->cacheIndex = nDof > 0 ? currentIndex : -1;
        linkIdCacheIndexMap[p.first] = p.second->inboundJoint->cacheIndex;
        currentIndex += nDof;
        if (nDof == 3) nSphericalJoint++;
        if (nDof == 1) nRevoluteJoint++;
    }

    assert(rootLink != nullptr);
}

int Articulation::GetNDof() const
{
    return (int)pxArticulation->getDofs();
}

const std::vector<int>& Articulation::GetJointDofsInIdOrder() const
{
    return jointDofs;
}

void Articulation::SetFixBaseFlag(bool shouldFixBase)
{
    pxArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, shouldFixBase);
}

int Articulation::GetNJoints() const
{
    return (int)jointList.size();
}

void Articulation::SetJointParams(std::vector<float>& target, const std::vector<float>& params)
{
    int index = 0;
    int nJoints = GetNJoints();
    for (int i = 0; i < nJoints; i++) {
        int jointDof = jointList[i]->nDof;
        for (int k = 0; k < jointDof; k++) {
            target[jointList[i]->cacheIndex + k] = params[index + k];
        }
        index += jointDof;
    }
}

vector<float> Articulation::GetJointPositionsQuaternion() const
{
    vector<float> result(7 + 4*nSphericalJoint + nRevoluteJoint);
    
    // transform from { x:up, y:back, z:right } to { x:front, y:up, z:right }
    PxQuat frameTransform(-PxPi / 2, PxVec3(0, 0, 1));

    PxTransform rootPose = rootLink->link->getGlobalPose();
    PxQuat rootRotation = rootPose.q * frameTransform;
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
            PxQuat rotation = frameTransform.getConjugate() * ConvertTwistSwingToQuaternion(
                mainCache->jointPosition[cacheIndex],
                mainCache->jointPosition[cacheIndex + 1],
                mainCache->jointPosition[cacheIndex + 2]
            ) * frameTransform;
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
    UniformQuaternion(rootGlobalRotation); // Never trust user input
    
    // transform from { x:front, y:up, z:right } to { x:up, y:back, z:right }
    PxQuat frameTransform(PxPi / 2, PxVec3(0, 0, 1));
    PxQuat rootPose = rootGlobalRotation * frameTransform;
    UniformQuaternion(rootPose);

    rootLink->link->setGlobalPose(PxTransform(rootGlobalTranslation, rootPose));

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
    
    // transform from { x:front, y:up, z:right } to { x:up, y:back, z:right }
    PxQuat frameTransform(PxPi / 2, PxVec3(0, 0, 1));

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

    // transform from { x:up, y:back, z:right } to { x:front, y:up, z:right }
    PxQuat frameTransform(-PxPi / 2, PxVec3(0, 0, 1));

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
            angularV = frameTransform.rotate(angularV);

            mainCache->jointVelocity[cacheIndex] = angularV.x;
            mainCache->jointVelocity[cacheIndex + 1] = angularV.y;
            mainCache->jointVelocity[cacheIndex + 2] = angularV.z;

            inputIndex += 4;
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eVELOCITY);
}

void Articulation::AddSPDForces(const std::vector<float>& targetPositions, float timeStep, bool applyRootExternalForce)
{
    int nDof = GetNDof();

    pxArticulation->commonInit();
    pxArticulation->computeGeneralizedMassMatrix(*massMatrixCache, true);

    pxArticulation->copyInternalStateToCache(*coriolisCache, PxArticulationCache::eVELOCITY);
    pxArticulation->computeCoriolisAndCentrifugalForce(*coriolisCache);

    pxArticulation->computeGeneralizedGravityForce(*gravityCache);
    pxArticulation->computeGeneralizedExternalForce(*externalForceCache);

    VectorXf centrifugalCoriolisGravityExternal(nDof);
    for (int i = 0; i < nDof; i++) {
        centrifugalCoriolisGravityExternal(i) = -(
            coriolisCache->jointForce[i] + 
            gravityCache->jointForce[i] + 
            externalForceCache->jointForce[i]
        );
    }
    MatrixXf H(nDof, nDof);
    for (int i = 0; i < nDof; i++) {
        for (int j = i; j < nDof; j++) {
            H(i, j) = H(j, i) = massMatrixCache->massMatrix[(i + 6) * (nDof + 6) + j + 6];
        }
    }

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    VectorXf proportionalTorquePlusQDotDeltaT(nDof);
    VectorXf derivativeTorque(nDof);

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

            PxQuat frameTransform(PxPi / 2, PxVec3(0, 0, 1));
            targetPosition = frameTransform.getConjugate() * targetPosition * frameTransform;

            if (targetPosition.w < 0) {
                targetPosition = -targetPosition;
            }

            PxQuat localRotation = ConvertTwistSwingToQuaternion(
                positions[cacheIndex],
                positions[cacheIndex + 1], 
                positions[cacheIndex + 2]
            );

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

            proportionalTorquePlusQDotDeltaT(cacheIndex) = proportionalForceInChildFrame[0] - 
                timeStep * velocities[cacheIndex] * kps[cacheIndex];
            proportionalTorquePlusQDotDeltaT(cacheIndex + 1) = proportionalForceInChildFrame[1] -
                timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
            proportionalTorquePlusQDotDeltaT(cacheIndex + 2) = proportionalForceInChildFrame[2] -
                timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

            derivativeTorque(cacheIndex) = -kds[cacheIndex] * velocities[cacheIndex];
            derivativeTorque(cacheIndex + 1) = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
            derivativeTorque(cacheIndex + 2) = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

            H(cacheIndex, cacheIndex) += kds[cacheIndex] * timeStep;
            H(cacheIndex + 1, cacheIndex + 1) += kds[cacheIndex + 1] * timeStep;
            H(cacheIndex + 2, cacheIndex + 2) += kds[cacheIndex + 2] * timeStep;

            targetPositionsIndex += 4;
        }
        else if (jointDof == 1) {
            proportionalTorquePlusQDotDeltaT(cacheIndex) = kps[cacheIndex] * (
                targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
                timeStep * velocities[cacheIndex]
            );
            derivativeTorque(cacheIndex) = -kds[cacheIndex] * velocities[cacheIndex];
            H(cacheIndex, cacheIndex) += kds[cacheIndex] * timeStep;

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

    VectorXf qDotDot = H.llt().solve(centrifugalCoriolisGravityExternal + proportionalTorquePlusQDotDeltaT + derivativeTorque);
    for (int i = 0; i < nDof; i++) {
        forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT(i) + derivativeTorque(i) - timeStep * kds[i] * qDotDot(i));
        if (forceLimits[i] > 0 && forces[i] > forceLimits[i]) {
            forces[i] = forceLimits[i];
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);

    if (applyRootExternalForce) {
        VectorXf p0c(6);
        for (int i = 0; i < 6; i++) {
            // Use coriolisCache by default. Any one is fine...
            p0c(i) = coriolisCache->jointForce[nDof + i];
        }
        MatrixXf F(6, nDof);
        for (int i = 0; i < 6; i++) {
            for (int j = 6; j < nDof + 6; j++) {
                F(i, j - 6) = massMatrixCache->massMatrix[i * (nDof + 6) + j];
            }
        }
        VectorXf FqDdotDot = F * qDotDot;
        MatrixXf I0c(6, 6);
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                I0c(i, j) = massMatrixCache->massMatrix[i * (nDof + 6) + j];
                printf("%f, ", I0c(i, j));
            }
            printf("\n");
        }
        
        VectorXf a0 = I0c.llt().solve(-p0c - FqDdotDot);
    }
}
vector<float> pred(34);
void Articulation::AddSPDForcesSparse(const std::vector<float>& targetPositions, float timeStep, bool applyRootExternalForce)
{
    int nDof = GetNDof();

    pxArticulation->commonInit();
    pxArticulation->computeGeneralizedMassMatrix(*massMatrixCache, false);

    pxArticulation->copyInternalStateToCache(*coriolisCache, PxArticulationCache::eVELOCITY);
    pxArticulation->computeCoriolisAndCentrifugalForce(*coriolisCache, true);

    pxArticulation->computeGeneralizedGravityForce(*gravityCache, true);
    pxArticulation->computeGeneralizedExternalForce(*externalForceCache, true);

    vector<float> centrifugalCoriolisGravityExternal(nDof + 6);
    for (int i = 0; i < nDof + 6; i++) {
        centrifugalCoriolisGravityExternal[(i + 6) % (nDof + 6)] = -(
            coriolisCache->jointForce[i] + 
            gravityCache->jointForce[i] + 
            externalForceCache->jointForce[i]
        );
    }

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    vector<float> proportionalTorquePlusQDotDeltaT(nDof + 6);
    vector<float> derivativeTorque(nDof + 6);

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

            PxQuat frameTransform(PxPi / 2, PxVec3(0, 0, 1));
            targetPosition = frameTransform.getConjugate() * targetPosition * frameTransform;

            if (targetPosition.w < 0) {
                targetPosition = -targetPosition;
            }

            PxQuat localRotation = ConvertTwistSwingToQuaternion(
                positions[cacheIndex],
                positions[cacheIndex + 1], 
                positions[cacheIndex + 2]
            );

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

            proportionalTorquePlusQDotDeltaT[cacheIndex + 6] = proportionalForceInChildFrame[0] - 
                timeStep * velocities[cacheIndex] * kps[cacheIndex];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 7] = proportionalForceInChildFrame[1] -
                timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 8] = proportionalForceInChildFrame[2] -
                timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

            derivativeTorque[cacheIndex + 6] = -kds[cacheIndex] * velocities[cacheIndex];
            derivativeTorque[cacheIndex + 7] = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
            derivativeTorque[cacheIndex + 8] = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

            for (int k = 0; k < 3; k++)
            {
                massMatrixCache->massMatrix[(cacheIndex + 6 + k) * (nDof + 7)] += kds[cacheIndex + k] * timeStep;
            }

            targetPositionsIndex += 4;
        }
        else if (jointDof == 1) {
            proportionalTorquePlusQDotDeltaT[cacheIndex + 6] = kps[cacheIndex] * (
                targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
                timeStep * velocities[cacheIndex]
            );
            derivativeTorque[cacheIndex + 6] = -kds[cacheIndex] * velocities[cacheIndex];
            massMatrixCache->massMatrix[(cacheIndex + 6) * (nDof + 7)] += kds[cacheIndex] * timeStep;

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

    float* rhs = new float[nDof + 6];
    for (int i = 0; i < nDof + 6; i++) {
        rhs[i] = centrifugalCoriolisGravityExternal[i] + proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i];
    }

    if (applyRootExternalForce) {
        for (int i = 0; i < 6; i++) {
            massMatrixCache->massMatrix[i * (nDof + 7)] += root_kds[i] * timeStep;
        }
        
        PxVec3 rootGlobalPosition = rootLink->link->getGlobalPose().p;
        PxQuat rootGlobalRotation = rootLink->link->getGlobalPose().q;

        PxVec3 rootGlobalLinearVelocity = rootLink->link->getLinearVelocity();
        
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

        for (int i = 0; i < 3; i++) {
            proportionalTorquePlusQDotDeltaT[i] = rootLocalProportionalLinearForcePlusQDotDeltaT[i];
            derivativeTorque[i] = rootLocalDerivativeLinearForce[i];
            rhs[i] += proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i];
        }
    }

    LTLInPlace(massMatrixCache->massMatrix, parentIndexMapForSparseLTL.data(), nDof + 6);
    backSubstitutionInPlace(massMatrixCache->massMatrix, rhs, parentIndexMapForSparseLTL.data(), nDof + 6);
    forwardSubstitutionInPlace(massMatrixCache->massMatrix, rhs, parentIndexMapForSparseLTL.data(), nDof + 6);

    printf("actual \n");
    for (int i = 6; i < 34; i++) {
        printf("%f, ", mainCache->jointAcceleration[i - 6]);
    }
    printf("\n");
    printf("predic \n");
    for (int i = 6; i < 34; i++) {
        printf("%f, ", pred[i]);
    }
    printf("\n");
    for (int i = 6; i < 34; i++) {
        pred[i] = rhs[i];
    }

    PxQuat rootGlobalRotation = rootLink->link->getGlobalPose().q;
    PxVec3 rootLocalLinearAcc(rhs[0], rhs[1], rhs[2]);
    PxVec3 rootLocalAngularAcc(rhs[3], rhs[4], rhs[5]);
    PxVec3 rootGlobalLinearAcc = rootGlobalRotation.rotate(rootLocalLinearAcc);
    PxVec3 rootGlobalAngularAcc = rootGlobalRotation.rotate(rootLocalAngularAcc);
    printf("root local linear\n");
    for (int i = 0; i < 3; i++) printf("%f, ", rootLocalLinearAcc[i]);
    printf("\n");
    printf("root local angular\n");
    for (int i = 0; i < 3; i++) printf("%f, ", rootLocalAngularAcc[i]);
    printf("\n");
    printf("root global linear\n");
    for (int i = 0; i < 3; i++) printf("%f, ", rootGlobalLinearAcc[i]);
    printf("\n");
    printf("root global angular\n");
    for (int i = 0; i < 3; i++) printf("%f, ", rootGlobalAngularAcc[i]);
    printf("\n");

    extern float g_ACC_test[6];
    printf("root actual acc\n");
    for (int i = 0; i < 6; i++) printf("%f, ", g_ACC_test[i]);
    printf("\n");

    for (int i = 0; i < nDof; i++) {
        forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT[i + 6] + derivativeTorque[i + 6]
            - timeStep * kds[i] * rhs[i + 6]);
        if (forceLimits[i] > 0 && forces[i] > forceLimits[i]) {
            forces[i] = forceLimits[i];
        }
    }

    if (applyRootExternalForce) {
        PxVec3 rootLocalExternalLinearForce;
        for (int i = 0; i < 3; i++) {
            rootLocalExternalLinearForce[i] = proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i]
                - timeStep * root_kds[i] * rhs[i];
        }

        extern float g_CRBA_RootExternalSpatialForce[6];
        for (int i = 0; i < 3; i++) {
            g_CRBA_RootExternalSpatialForce[i] = rootLocalExternalLinearForce[i];
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);
    delete rhs;
}

void Articulation::AddSPDForcesABA(const std::vector<float>& targetPositions, float timeStep)
{
    int nDof = GetNDof();

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    vector<float> proportionalTorquePlusQDotDeltaT(nDof);
    vector<float> derivativeTorque(nDof);

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

            PxQuat frameTransform(PxPi / 2, PxVec3(0, 0, 1));
            targetPosition = frameTransform.getConjugate() * targetPosition * frameTransform;

            if (targetPosition.w < 0) {
                targetPosition = -targetPosition;
            }

            PxQuat localRotation = ConvertTwistSwingToQuaternion(
                positions[cacheIndex],
                positions[cacheIndex + 1], 
                positions[cacheIndex + 2]
            );

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

            proportionalTorquePlusQDotDeltaT[cacheIndex] = proportionalForceInChildFrame[0] - 
                timeStep * velocities[cacheIndex] * kps[cacheIndex];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 1] = proportionalForceInChildFrame[1] -
                timeStep * velocities[cacheIndex + 1] * kps[cacheIndex + 1];
            proportionalTorquePlusQDotDeltaT[cacheIndex + 2] = proportionalForceInChildFrame[2] -
                timeStep * velocities[cacheIndex + 2] * kps[cacheIndex + 2];

            derivativeTorque[cacheIndex] = -kds[cacheIndex] * velocities[cacheIndex];
            derivativeTorque[cacheIndex + 1] = -kds[cacheIndex + 1] * velocities[cacheIndex + 1];
            derivativeTorque[cacheIndex + 2] = -kds[cacheIndex + 2] * velocities[cacheIndex + 2];

            targetPositionsIndex += 4;
        }
        else if (jointDof == 1) {
            proportionalTorquePlusQDotDeltaT[cacheIndex] = kps[cacheIndex] * (
                targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
                timeStep * velocities[cacheIndex]
            );
            derivativeTorque[cacheIndex] = -kds[cacheIndex] * velocities[cacheIndex];

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

    for (int i = 0; i < nDof; i++) {
        forces[i] = proportionalTorquePlusQDotDeltaT[i] + derivativeTorque[i];
        if (forceLimits[i] > 0 && forces[i] > forceLimits[i]) {
            forces[i] = forceLimits[i];
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);
#ifdef ENABLE_SPD_ABA
    extern float  			g_SPD_Dt;
    extern const float* 	g_SPD_Kd;
    extern const int*		g_SPD_LinkIdCacheIndexMap;

    g_SPD_Dt = timeStep;
    g_SPD_Kd = kds.data();
    g_SPD_LinkIdCacheIndexMap = linkIdCacheIndexMap.data();
#endif
}

void Articulation::FetchKinematicData() const
{
    pxArticulation->copyInternalStateToCache(*mainCache, PxArticulationCache::eALL);
}

const Link* Articulation::GetLinkByName(std::string name) const
{
    const auto& it = linkMap.find(name);
    if (it == linkMap.end()) {
        printf("Error: Link with name %s not found.\n", name.c_str());
        assert(false);
        return nullptr;
    }
    return it->second;
}

const Joint* Articulation::GetJointByName(std::string name) const
{
    const auto& it = jointMap.find(name);
    if (it == jointMap.end()) {
        printf("Error: Joint with name %s not found.\n", name.c_str());
        assert(false);
        return nullptr;
    }
    return it->second;
}

const vector<Joint*>& Articulation::GetAllJointsInIdOrder() const
{
    return jointList;
}

const Link* Articulation::GetRootLink() const 
{
    return rootLink;
}

void Articulation::SetKPs(const std::vector<float>& kps)
{
    SetJointParams(this->kps, kps);
}

void Articulation::SetKDs(const std::vector<float>& kds)
{
    SetJointParams(this->kds, kds);
}

void Articulation::SetForceLimits(const std::vector<float>& forceLimits)
{
    SetJointParams(this->forceLimits, forceLimits);
}
