#include "Scene.h"
#include "Foundation.h"

using namespace physx;

SceneDescription::SceneDescription()
{
    gravity = -9.81f;
    nWorkerThreads = 0;
    enableGPUDynamics = false;
    enableGPUBroadPhase = false;
}

SceneObject::SceneObject(PxActor* pxActor)
{
    this->pxActor = pxActor;
}

static PxFilterFlags CollisionShader(
	PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* /*constantBlock*/, PxU32 /*constantBlockSize*/)
{
	// let triggers through
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}
	// generate contacts for all that were not filtered above
	pairFlags = PxPairFlag::eCONTACT_DEFAULT;

	// trigger the contact callback for pairs (A,B) where 
	// the filtermask of A contains the ID of B and vice versa.
	if ((filterData0.word0 & filterData1.word1) && (filterData1.word0 & filterData0.word1))
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;

	return PxFilterFlag::eDEFAULT;
}

Scene::Scene(SceneDescription description, float timeStep)
{
	const Foundation* foundation = Foundation::GetFoundation();
    PxSceneDesc pxSceneDesc(foundation->GetPxPhysics()->getTolerancesScale());

    pxSceneDesc.gravity = PxVec3(0.f, description.gravity, 0.f);
    pxSceneDesc.cudaContextManager = foundation->GetPxCudaContextManager();
    pxSceneDesc.solverType = PxSolverType::eTGS;
    pxSceneDesc.filterShader = CollisionShader;
	pxSceneDesc.simulationEventCallback = this;

    pxCpuDispatcher = PxDefaultCpuDispatcherCreate(description.nWorkerThreads);
    pxSceneDesc.cpuDispatcher = pxCpuDispatcher;

	if (description.enableGPUDynamics)
		pxSceneDesc.flags |= PxSceneFlag::eENABLE_GPU_DYNAMICS;
	if (description.enableGPUBroadPhase)
		pxSceneDesc.broadPhaseType = PxBroadPhaseType::eGPU;
	
	pxScene = foundation->GetPxPhysics()->createScene(pxSceneDesc);
	this->timeStep = timeStep;
}

Scene::~Scene()
{
}

void Scene::Dispose()
{
    pxScene->release();
    pxCpuDispatcher->release();
}

void Scene::AddObject(SceneObject obj)
{
    pxScene->addActor(*obj.pxActor);
}

void Scene::AddArticulation(Articulation* articulation)
{
	pxScene->addArticulation(*articulation->GetPxArticulation());
	articulation->InitControl(); // TODO: make init control a general function
}

void Scene::Step()
{
	// TODO: remove this
	int contactFlag;
	extern void control(PxReal dt, int /*contactFlag*/);
	control(timeStep, contactFlag);

    pxScene->simulate(timeStep);
	pxScene->fetchResults(true);
}

PxScene* Scene::GetPxScene() const
{
	return pxScene;
}

void Scene::onContact(const PxContactPairHeader &pairHeader, 
    const PxContactPair *pairs, PxU32 nbPairs) 
{
    for (PxU32 i = 0; i < nbPairs; i++)
    {
        const PxContactPair& cp = pairs[i];

        if (cp.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
        {
            ReportContact(pairHeader.actors[0], pairHeader.actors[1]);
        }
    }
}

void Scene::ReportContact(const physx::PxActor* actor0, const physx::PxActor* actor1)
{
	// TODO
	printf("report contact\n");
}
