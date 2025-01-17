#include "Articulation.h"
#include "Foundation.h"
#include "CommonMath.h"
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
            PxVec3 jointPosition(
                mainCache->jointPosition[cacheIndex],
                mainCache->jointPosition[cacheIndex + 1],
                mainCache->jointPosition[cacheIndex + 2]
            );
            PxQuat jointQuat(jointPosition.magnitude(), jointPosition.getNormalized());
            PxQuat rotation = frameTransform.getConjugate() * jointQuat * frameTransform;
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

            PxVec3 axis;
            float angle;
            rotation.toRadiansAndUnitAxis(angle, axis);
            axis *= angle;
            
            mainCache->jointPosition[cacheIndex] = axis.x;
            mainCache->jointPosition[cacheIndex + 1] = axis.y;
            mainCache->jointPosition[cacheIndex + 2] = axis.z;

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

void Articulation::AddSPDForces(const std::vector<float>& targetPositions, float timeStep)
{
    int nDof = GetNDof();

    pxArticulation->commonInit();
    pxArticulation->computeGeneralizedMassMatrix(*massMatrixCache);

    pxArticulation->copyInternalStateToCache(*coriolisCache, PxArticulationCache::eVELOCITY);
    pxArticulation->computeCoriolisAndCentrifugalForce(*coriolisCache);

    pxArticulation->computeGeneralizedGravityForce(*gravityCache);
    pxArticulation->computeGeneralizedExternalForce(*externalForceCache);

    VectorXd centrifugalCoriolisGravityExternal(nDof);
    for (int i = 0; i < nDof; i++) {
        centrifugalCoriolisGravityExternal(i) = -(
            coriolisCache->jointForce[i] + 
            gravityCache->jointForce[i] + 
            externalForceCache->jointForce[i]
        );
    }
    MatrixXd H(nDof, nDof);
    for (int i = 0; i < nDof; i++) {
        for (int j = i; j < nDof; j++) {
            H(i, j) = H(j, i) = massMatrixCache->massMatrix[i * nDof + j];
        }
    }

    PxReal *positions = mainCache->jointPosition;
    PxReal *velocities = mainCache->jointVelocity;
    PxReal *forces = mainCache->jointForce;

    VectorXd proportionalTorquePlusQDotDeltaT(nDof);
    VectorXd derivativeTorque(nDof);

    int nJoints = GetNJoints();
    int targetPositionsIndex = 0;

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

            PxVec3 jointVar(positions[cacheIndex], positions[cacheIndex + 1], positions[cacheIndex + 2]);
            PxQuat localRotation(jointVar.magnitude(), jointVar.getNormalized());
           
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

    VectorXd qDotDot = H.llt().solve(centrifugalCoriolisGravityExternal + proportionalTorquePlusQDotDeltaT + derivativeTorque);
    for (int i = 0; i < nDof; i++) {
        forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT(i) + derivativeTorque(i) - timeStep * kds[i] * qDotDot(i));
        if (forceLimits[i] > 0 && PxAbs(forces[i]) > forceLimits[i]) {
            forces[i] = forceLimits[i] * PxSign(forces[i]);
        }
    }

    pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);
}

vector<float> Articulation::GetCurrentForces() const
{
    int nDof = GetNDof();
    vector<float> forces(nDof);
    for (int i = 0; i < nDof; i++) {
        forces[i] = mainCache->jointForce[i];
    }
    return forces;
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
    int targetPositionsIndex = 0;

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

            PxVec3 jointVar(positions[cacheIndex], positions[cacheIndex + 1], positions[cacheIndex + 2]);
            PxQuat localRotation(jointVar.magnitude(), jointVar.getNormalized());

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
