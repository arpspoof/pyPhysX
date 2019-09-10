
#include <ctype.h>
#include <vector>
#include <stdio.h>
#include <chrono>

#include "PxPhysicsAPI.h"
#include "config.h"
#include "ArticulationTree.h"

#include <iostream>
#include <vector>
#include <algorithm>

#include "Foundation.h"
#include "Scene.h"

using namespace physx;
using namespace std;
using namespace std::chrono;

static int contactFlag = 0;

Articulation* articulation;

void setupFiltering(PxRigidActor* actor, PxU32 filterGroup, PxU32 filterMask)
{
	PxFilterData filterData;
	filterData.word0 = filterGroup; // word0 = own ID
	filterData.word1 = filterMask;  // word1 = ID mask to filter pairs that trigger a
									// contact callback;
	const PxU32 numShapes = actor->getNbShapes();
	PxShape** shapes = (PxShape**)malloc(sizeof(PxShape*)*numShapes);
	actor->getShapes(shapes, numShapes);
	for (PxU32 i = 0; i < numShapes; i++)
	{
		PxShape* shape = shapes[i];
		shape->setSimulationFilterData(filterData);
	}
	free(shapes);
}

Scene* scene;
PxMaterial* material;

void loadRoot();
void InitControl();

void initPhysics(bool /*interactive*/)
{
	scene = new Scene(SceneDescription());

	PxPhysics* physics = Foundation::GetFoundation()->GetPxPhysics();
	material = physics->createMaterial(getConfigF("C_STATIC_FRICTION"), getConfigF("C_DYNAMIC_FRICTION"), 0.f);

	if (getConfigI("S_GROUND")) {
		PxRigidStatic* groundPlane = PxCreatePlane(*physics, PxPlane(0, 1, 0, 0), *material);
		SceneObject ground(groundPlane);
		scene->AddObject(ground);
	}
	
	articulation = new Articulation();

	loadRoot();

	scene->AddArticulation(articulation);
	scene->timeStep = getConfigF("C_TIME_STEP");

	InitControl();
}
	
void cleanupPhysics(bool /*interactive*/)
{
	articulation->Dispose();
	scene->Dispose();
	delete articulation;
	delete scene;
}

PxReal motions[98][36];

#include <fstream>
#include "GlutRenderer.h"

int xFrame = 0;

void keyHandler(unsigned char key)
{
	switch (key) {
	case '1':
		writeConfigFile();
		break;
	case 'z': xFrame = PxMax(0, xFrame - 1); printf("xframe = %d\n", xFrame); break;
	case 'x': xFrame = PxMin(95, xFrame + 1); printf("xframe = %d\n", xFrame); break;
	}
}

int main(int argc, char** argv)
{
	if (argc > 1) {
		const char* config_path = argv[1];
		readConfigFile(config_path);
	}
	else {
		printf("no config file specified\n");
	}

	ifstream motioninput("/home/zhiqiy/tmp/testMotion2.txt");
	for (int i = 0; i < 32; i++) {
		for (int j = 0; j < 43; j++) {
			PxReal tmp;motioninput >> tmp;
			if (j < 7) continue;
			motions[i][j - 7] = tmp;
		}
	}
	motioninput.close();

#if 1
	initPhysics(false);
	auto renderer = glutRenderer::GlutRenderer::GetInstance();
	renderer->AttachScene(scene, keyHandler);
	renderer->StartRenderLoop();
	cleanupPhysics(false);
#else
	static const PxU32 frameCount = 10000;
	initPhysics(false);
    auto starttime = high_resolution_clock::now();
	for(PxU32 i=0; i<frameCount; i++)
		scene->Step();
    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    printf("%lld\n", duration);
	cleanupPhysics(false);
#endif
	return 0;
}
