
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
		scene->CreatePlane(material, vec3(0, 1, 0), 0);
	}

	ArticulationTree arTree;

	NULLLinkBody bodyBase;
	SphereLinkBody bodyRoot(6.f, 0.36f, material);
	SphereLinkBody bodyChest(14.f, 0.48f, material);
	SphereLinkBody bodyNeck(2.f, 0.41f, material);
	CapsuleLinkBody bodyHip(4.5f, 0.22f, 1.2f, material);
	CapsuleLinkBody bodyKnee(3.f, 0.2f, 1.24f, material);
	CapsuleLinkBody bodyShoulder(1.5f, 0.18f, 0.72f, material);
	CapsuleLinkBody bodyElbow(1.f, 0.16f, 0.54f, material);
	SphereLinkBody bodyWrist(0.5f, 0.16f, material);
	BoxLinkBody bodyAnkle(1.0f, 0.22f, 0.708f, 0.36f, material); // note: switch x and y

	// Descriptions
	NULLDescriptionNode descrBase("base", &bodyBase);
	FixedDescriptionNode descrRoot("root", "root", &bodyRoot, 
		PxVec3(0, 0.28f, 0), PxVec3(0, 0, 0));
	SpericalDescriptionNode descrChest("chest", "chest", &bodyChest, 
		PxVec3(0, 0.48f, 0), PxVec3(0, 0.944604f, 0));
	SpericalDescriptionNode descrNeck("neck", "neck", &bodyNeck, 
		PxVec3(0, 0.7f, 0), PxVec3(0, 0.895576f, 0));
	SpericalDescriptionNode descrRHip("right_hip", "right_hip", &bodyHip, 
		PxVec3(0, -0.84f, 0), PxVec3(0, 0, 0.339548f));
	SpericalDescriptionNode descrLHip("left_hip", "left_hip", &bodyHip, 
		PxVec3(0, -0.84f, 0), PxVec3(0, 0, -0.339548f));
	RevoluteDescriptionNode descrRKnee("right_knee", "right_knee", &bodyKnee, PxArticulationAxis::eSWING2,
		PxVec3(0, -0.8f, 0), PxVec3(0, -1.686184f, 0));
	RevoluteDescriptionNode descrLKnee("left_knee", "left_knee", &bodyKnee, PxArticulationAxis::eSWING2,
		PxVec3(0, -0.8f, 0), PxVec3(0, -1.686184f, 0));
	SpericalDescriptionNode descrRShoulder("right_shoulder", "right_shoulder", &bodyShoulder,
		PxVec3(0, -0.56f, 0), PxVec3(-0.096200f, 0.974000f, 0.732440f));
	SpericalDescriptionNode descrLShoulder("left_shoulder", "left_shoulder", &bodyShoulder,
		PxVec3(0, -0.56f, 0), PxVec3(-0.096200f, 0.974000f, -0.732440f));
	RevoluteDescriptionNode descrRElbow("right_elbow", "right_elbow", &bodyElbow, PxArticulationAxis::eSWING2,
		PxVec3(0, -0.48f, 0), PxVec3(0, -1.099152f, 0));
	RevoluteDescriptionNode descrLElbow("left_elbow", "left_elbow", &bodyElbow, PxArticulationAxis::eSWING2,
		PxVec3(0, -0.48f, 0), PxVec3(0, -1.099152f, 0));
	FixedDescriptionNode descrRWrist("right_wrist", "right_wrist", &bodyWrist,
		PxVec3(0, 0, 0), PxVec3(0, -1.035788f, 0));
	FixedDescriptionNode descrLWrist("left_wrist", "left_wrist", &bodyWrist,
		PxVec3(0, 0, 0), PxVec3(0, -1.035788f, 0));
	SpericalDescriptionNode descrRAnkle("right_ankle", "right_ankle", &bodyAnkle,
		PxVec3(0.18f, -0.09f, 0), PxVec3(0, -1.63948f, 0));
	SpericalDescriptionNode descrLAnkle("left_ankle", "left_ankle", &bodyAnkle,
		PxVec3(0.18f, -0.09f, 0), PxVec3(0, -1.63948f, 0));

	arTree.AddNULLDescriptionNode(descrBase);
	arTree.SetRoot("base");

	arTree.AddFixedDescriptionNode(descrRoot);
	arTree.Connect("base", "root");

	arTree.AddSpericalDescriptionNode(descrChest);
	arTree.Connect("root", "chest");

	arTree.AddSpericalDescriptionNode(descrNeck);
	arTree.Connect("chest", "neck");

	arTree.AddSpericalDescriptionNode(descrRHip);
	arTree.Connect("root", "right_hip");

	arTree.AddSpericalDescriptionNode(descrLHip);
	arTree.Connect("root", "left_hip");

	arTree.AddRevoluteDescriptionNode(descrRKnee);
	arTree.Connect("right_hip", "right_knee");

	arTree.AddRevoluteDescriptionNode(descrLKnee);
	arTree.Connect("left_hip", "left_knee");

	arTree.AddSpericalDescriptionNode(descrRAnkle);
	arTree.Connect("right_knee", "right_ankle");

	arTree.AddSpericalDescriptionNode(descrLAnkle);
	arTree.Connect("left_knee", "left_ankle");

	arTree.AddSpericalDescriptionNode(descrRShoulder);
	arTree.Connect("chest", "right_shoulder");

	arTree.AddSpericalDescriptionNode(descrLShoulder);
	arTree.Connect("chest", "left_shoulder");

	arTree.AddRevoluteDescriptionNode(descrRElbow);
	arTree.Connect("right_shoulder", "right_elbow");

	arTree.AddRevoluteDescriptionNode(descrLElbow);
	arTree.Connect("left_shoulder", "left_elbow");

	arTree.AddFixedDescriptionNode(descrRWrist);
	arTree.Connect("right_elbow", "right_wrist");

	arTree.AddFixedDescriptionNode(descrLWrist);
	arTree.Connect("left_elbow", "left_wrist");
	
	articulation = scene->CreateArticulation(&arTree, vec3(0, 33.75f, 0));

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
