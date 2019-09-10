#include "Articulation.h"
#include "Foundation.h"
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

Articulation::Articulation()
{
    pxArticulation = Foundation::GetFoundation()->GetPxPhysics()->createArticulationReducedCoordinate();
}

Articulation::~Articulation() {
    for (auto& it : linkMap) {
        delete it.second;
    }
    for (auto& it : jointMap) {
        delete it.second;
    }
}

void Articulation::InitControl()
{
    AssignIndices();

    mainCache = pxArticulation->createCache();
    massMatrixCache = pxArticulation->createCache();
    coriolisCache = pxArticulation->createCache();
    gravityCache = pxArticulation->createCache();
    externalForceCache = pxArticulation->createCache();

    int nDof = GetNDof();
    kps = vector<float>(nDof, 0);
    kds = vector<float>(nDof, 0);
    forceLimits = vector<float>(nDof, -1);

	typedef tuple<int, Joint*, string> PIJN;
	vector<PIJN> joints;
	
	for (auto &kvp : jointMap) {
		if (kvp.second->nDof >= 1) {
			joints.push_back(make_tuple(kvp.second->cacheIndex, kvp.second, kvp.first));
		}
	}

	sort(joints.begin(), joints.end(), [](PIJN &a, PIJN &b) { return get<0>(a) < get<0>(b); });

	for (PIJN &pijn : joints) {
		jointList.push_back(get<1>(pijn));
		jointDofs.push_back(get<1>(pijn)->nDof);
		jointNames.push_back(get<2>(pijn));
	}
}

void Articulation::AssignIndices() {
	typedef pair<PxU32, Link*> PIDL;
	vector<PIDL> linkIndices;
	for (auto &kvp : linkMap) {
		linkIndices.push_back(make_pair(kvp.second->link->getLinkIndex(), kvp.second));
	}
	sort(linkIndices.begin(), linkIndices.end(), [=](PIDL a, PIDL b) { return a.first < b.first; });

	int currentIndex = 0;
	for (PIDL &p : linkIndices) {
		int nDof = (int)p.second->link->getInboundJointDof();
		if (!p.second->inboundJoint) {
			continue;
		}
		p.second->inboundJoint->nDof = nDof;
		p.second->inboundJoint->cacheIndex = currentIndex;
		currentIndex += nDof;
		printf("link id = %d, dof = %d, index = %d\n", p.first, nDof, p.second->inboundJoint->cacheIndex);
	}
}

void Articulation::Dispose() 
{
    pxArticulation->release();
}

PxArticulationReducedCoordinate* Articulation::GetPxArticulation() const
{
    return pxArticulation;
}

int Articulation::GetNDof() const
{
    return (int)pxArticulation->getDofs();
}

const int* Articulation::GetJointDofsInIdOrder() const
{
	return jointDofs.data();
}

string Articulation::GetJointNameById(int id) const
{
	return jointNames[id];
}

void Articulation::SetFixBaseFlag(bool shouldFixBase)
{
    pxArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, shouldFixBase);
}

void Articulation::SetKPs(const float kps[])
{
    int nDof = GetNDof();
    for (int i = 0; i < nDof; i++)
        this->kps[i] = kps[i];
}

void Articulation::SetKDs(const float kds[])
{
    int nDof = GetNDof();
    for (int i = 0; i < nDof; i++)
        this->kds[i] = kds[i];
}

void Articulation::SetForceLimits(const float forceLimits[])
{
    int nDof = GetNDof();
    for (int i = 0; i < nDof; i++)
        this->forceLimits[i] = forceLimits[i];
}

static PxQuat ConvertTwistSwingToQuaternion(float t, float s1, float s2)
{
    PxVec3 s(0, s1, s2);
	PxQuat swingQuat = s.isZero() ? PxQuat(0, 0, 0, 1) : PxQuat(s.magnitude(), s.getNormalized());
	return swingQuat * PxQuat(t, PxVec3(1, 0, 0));
}

void Articulation::AddSPDForces(const float targetPositions[], float timeStep)
{
	int nDof = GetNDof();

    pxArticulation->copyInternalStateToCache(*mainCache, PxArticulationCache::eALL);

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

	memset(forces, 0, sizeof(PxReal) * nDof);

	VectorXd proportionalTorquePlusQDotDeltaT(nDof);
	VectorXd derivativeTorque(nDof);

	int nActiveJoints = (int)jointDofs.size();

	int cacheIndex = 0;
	int targetPositionsIndex = 0;

	for (int i = 0; i < nActiveJoints; i++) {
		int jointDof = jointDofs[i];
		if (jointDof == 3) {
			PxQuat targetPosition(
				targetPositions[targetPositionsIndex + 1],
				targetPositions[targetPositionsIndex + 2],
				targetPositions[targetPositionsIndex + 3],
				targetPositions[targetPositionsIndex]
			);
			PxVec3 kp(
				kps[cacheIndex],
				kps[cacheIndex + 1],
				kps[cacheIndex + 2]
			);

			PxQuat frameTransform(-PxPi / 2, PxVec3(0, 0, 1));
			targetPosition = frameTransform * targetPosition * frameTransform.getConjugate();

			if (targetPosition.w < 0) {
				targetPosition = -targetPosition;
			}

			PxQuat localRotation = ConvertTwistSwingToQuaternion(
				positions[cacheIndex],
				positions[cacheIndex + 1], 
				positions[cacheIndex + 2]
			);

			PxQuat posDifference = targetPosition * localRotation.getConjugate();
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

			cacheIndex += 3;
			targetPositionsIndex += 4;
		}
		else if (jointDof == 1) {
			proportionalTorquePlusQDotDeltaT(cacheIndex) = kps[cacheIndex] * (
				targetPositions[targetPositionsIndex] - positions[cacheIndex] - 
				timeStep * velocities[cacheIndex]
			);
			derivativeTorque(cacheIndex) = -kds[cacheIndex] * velocities[cacheIndex];
			H(cacheIndex, cacheIndex) += kds[cacheIndex] * timeStep;

			cacheIndex += 1;
			targetPositionsIndex += 1;
		}
		else {
			printf("no controller defined for Dof %d\n", jointDof);
			assert(false);
		}
	}

	VectorXd qDotDot = H.llt().solve(centrifugalCoriolisGravityExternal + proportionalTorquePlusQDotDeltaT + derivativeTorque);
	for (PxU32 i = 0; i < nDof; i++) {
		forces[i] = (PxReal)(proportionalTorquePlusQDotDeltaT(i) + derivativeTorque(i) - timeStep * kds[i] * qDotDot(i));
		if (forceLimits[i] > 0 && forces[i] > forceLimits[i]) {
			forces[i] = forceLimits[i];
		}
	}

	pxArticulation->applyCache(*mainCache, PxArticulationCache::eFORCE);
}
