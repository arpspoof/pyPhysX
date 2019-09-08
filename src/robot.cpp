#include "robot.h"
#include "config.h"
#include "globals.h"
#include "articulationTree.h"

#include <iostream>
#include <string>

using namespace physx;
using namespace std;

extern Articulation ar;

// Link bodies
NULLLinkBody bodyBase;
SphereLinkBody bodyRoot(6.f, 0.36f);
SphereLinkBody bodyChest(14.f, 0.48f);
SphereLinkBody bodyNeck(2.f, 0.41f);
CapsuleLinkBody bodyHip(4.5f, 0.22f, 1.2f);
CapsuleLinkBody bodyKnee(3.f, 0.2f, 1.24f);
CapsuleLinkBody bodyShoulder(1.5f, 0.18f, 0.72f);
CapsuleLinkBody bodyElbow(1.f, 0.16f, 0.54f);
SphereLinkBody bodyWrist(0.5f, 0.16f);
BoxLinkBody bodyAnkle(1.0f, 0.22f, 0.708f, 0.36f); // note: switch x and y

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

void loadRoot() {
	ArticulationTree arTree;

	arTree.addNULLDescriptionNode(descrBase);
	arTree.setRoot("base");

	arTree.addFixedDescriptionNode(descrRoot);
	arTree.connect("base", "root");

	arTree.addSpericalDescriptionNode(descrChest);
	arTree.connect("root", "chest");

	arTree.addSpericalDescriptionNode(descrNeck);
	arTree.connect("chest", "neck");

	arTree.addSpericalDescriptionNode(descrRHip);
	arTree.connect("root", "right_hip");

	arTree.addSpericalDescriptionNode(descrLHip);
	arTree.connect("root", "left_hip");

	arTree.addRevoluteDescriptionNode(descrRKnee);
	arTree.connect("right_hip", "right_knee");

	arTree.addRevoluteDescriptionNode(descrLKnee);
	arTree.connect("left_hip", "left_knee");

	arTree.addSpericalDescriptionNode(descrRAnkle);
	arTree.connect("right_knee", "right_ankle");

	arTree.addSpericalDescriptionNode(descrLAnkle);
	arTree.connect("left_knee", "left_ankle");

	arTree.addSpericalDescriptionNode(descrRShoulder);
	arTree.connect("chest", "right_shoulder");

	arTree.addSpericalDescriptionNode(descrLShoulder);
	arTree.connect("chest", "left_shoulder");

	arTree.addRevoluteDescriptionNode(descrRElbow);
	arTree.connect("right_shoulder", "right_elbow");

	arTree.addRevoluteDescriptionNode(descrLElbow);
	arTree.connect("left_shoulder", "left_elbow");

	arTree.addFixedDescriptionNode(descrRWrist);
	arTree.connect("right_elbow", "right_wrist");

	arTree.addFixedDescriptionNode(descrLWrist);
	arTree.connect("left_elbow", "left_wrist");

	arTree.buildArticulation(ar);

	gArticulation->setArticulationFlag(PxArticulationFlag::Enum::eFIX_BASE, true);

	auto rightFoot = ar.linkMap["right_ankle"]->link;
	auto leftFoot = ar.linkMap["left_ankle"]->link;
	setupFiltering(rightFoot, CollisionGroup::RightFoot, CollisionGroup::Ground);
	setupFiltering(leftFoot, CollisionGroup::LeftFoot, CollisionGroup::Ground);
}
