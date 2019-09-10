#include "Articulation.h"
#include "Foundation.h"

#include <algorithm>

using namespace physx;
using namespace std;

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

void Articulation::SetFixBaseFlag(bool shouldFixBase)
{
    pxArticulation->setArticulationFlag(PxArticulationFlag::eFIX_BASE, shouldFixBase);
}
