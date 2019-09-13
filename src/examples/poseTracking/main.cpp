
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

#include "MathInterface.h"
#include "PrimitiveObjects.h"
#include "Foundation.h"
#include "UrdfLoader.h"
#include "Scene.h"
#include "Actor.h"

using namespace physx;
using namespace std;
using namespace std::chrono;

Foundation* foundation;
Articulation* articulation;
Scene* scene;
Material* material;

void InitControl();

void initPhysics(bool /*interactive*/)
{
	foundation = new Foundation();
	scene = foundation->CreateScene(SceneDescription(), 0.001f);

	material = scene->CreateMaterial(1.f, 1.f, 0.f);

	if (getConfigI("S_GROUND")) {
		auto plane = scene->CreatePlane(material, vec3(0, 1, 0), 0);
		plane->setupCollisionFiltering(1, 2 | 4);
	}
	
	articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 3.75f, 0));

	articulation->linkMap["right_ankle"]->setupCollisionFiltering(2, 1 | 4);
	articulation->linkMap["left_ankle"]->setupCollisionFiltering(4, 1 | 2);

	for (auto &kvp : articulation->jointMap) {
		printf("%s: %d\n", kvp.first.c_str(), kvp.second->cacheIndex);
	}

	InitControl();
}
	
void cleanupPhysics(bool /*interactive*/)
{
	foundation->Dispose();
	delete foundation;
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

extern void control(PxReal dt);

void beforeRender()
{
/*	auto contacts = scene->GetAllContactPairs();
	for (auto &p: contacts) {
		printf("%d, %d\n", p.first, p.second);
	}*/
	control(scene->timeStep);
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
	renderer->AttachScene(scene, keyHandler, beforeRender);
	renderer->StartRenderLoop();
	cleanupPhysics(false);
#else
	static const PxU32 frameCount = 10000;
	initPhysics(false);
    auto starttime = high_resolution_clock::now();
	for(PxU32 i=0; i<frameCount; i++) {
		control(scene->timeStep);
		scene->Step();
	}
    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    printf("%lld\n", duration);
	cleanupPhysics(false);
#endif
	return 0;
}
