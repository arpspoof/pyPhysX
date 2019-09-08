#include "globals.h"
#include "config.h"
#include "robot.h"

void loader() {
	loadRoot();
}

PxArticulationJointReducedCoordinate* joint12;
PxArticulationLink* link1;
PxArticulationLink* link2;

void setKPKD() {
	PxReal kp = getConfigF("T_KP");
	PxReal kd = getConfigF("T_KD");

	joint12->setDrive(PxArticulationAxis::eTWIST, kp, kd, 100);
	joint12->setDrive(PxArticulationAxis::eSWING1, kp, kd, 100);
	joint12->setDrive(PxArticulationAxis::eSWING2, kp, kd, 100);
}

void loader5() {
	PxQuat rtz2pi(PxPi / 2, PxVec3(0, 0, 1));

	PxArticulationLink* base = gArticulation->createLink(NULL, PxTransform(PxVec3(0.f, 0.25f, 0.f), rtz2pi));
	PxRigidActorExt::createExclusiveShape(*base, PxBoxGeometry(0.25f, 0.5f, 0.5f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*base, 30.f);

	link1 = gArticulation->createLink(base, PxTransform(PxVec3(0.f, 1.f, 0.f), rtz2pi));
	PxRigidActorExt::createExclusiveShape(*link1, PxBoxGeometry(0.5f, 0.25f, 0.25f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*link1, 3.f);

	PxArticulationJointReducedCoordinate* joint01 = 
		static_cast<PxArticulationJointReducedCoordinate*>(link1->getInboundJoint());
	joint01->setJointType(PxArticulationJointType::eFIX);
	joint01->setParentPose(PxTransform(PxVec3(0.25f, 0.f, 0.f)));
	joint01->setChildPose(PxTransform(PxVec3(-0.5f, 0.f, 0.f)));

	link2 = gArticulation->createLink(link1, PxTransform(PxVec3(0.f, 2.5f, 0.f), rtz2pi));
	PxRigidActorExt::createExclusiveShape(*link2, PxBoxGeometry(1.f, 0.1f, 0.1f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*link2, 3.f);

	joint12 = static_cast<PxArticulationJointReducedCoordinate*>(link2->getInboundJoint());
	joint12->setJointType(PxArticulationJointType::eSPHERICAL);
	joint12->setParentPose(PxTransform(PxVec3(0.5f, 0.f, 0.f)));
	joint12->setChildPose(PxTransform(PxVec3(-1.f, 0.f, 0.f)));
	joint12->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
	joint12->setMotion(PxArticulationAxis::eSWING1, PxArticulationMotion::eFREE);
	joint12->setMotion(PxArticulationAxis::eSWING2, PxArticulationMotion::eFREE);

	setKPKD();
	
/*	link2->setLinearDamping(0.2f);
	link2->setAngularDamping(0.2f);

	link2->setMaxAngularVelocity(20.f);
	link2->setMaxLinearVelocity(100.f);*/

	gScene->addArticulation(*gArticulation);
}

void loader3() {
	const PxReal runnerLength = 2.f;
	const PxReal placementDistance = 1.8f;

	const PxReal cosAng = (placementDistance) / (runnerLength);

	const PxReal angle = PxAcos(cosAng);

	const PxReal sinAng = PxSin(angle);

	const PxQuat leftRot(-angle, PxVec3(1.f, 0.f, 0.f));
	const PxQuat rightRot(angle, PxVec3(1.f, 0.f, 0.f));

	//(1) Create base...
	PxArticulationLink* base = gArticulation->createLink(NULL, PxTransform(PxVec3(0.f, 0.25f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*base, PxBoxGeometry(5.5f, 0.25f, 5.5f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*base, 300.f);

	//Now create the slider and fixed joints...

	gArticulation->setSolverIterationCounts(32);

	PxArticulationLink* leftRoot = gArticulation->createLink(base, PxTransform(PxVec3(0.f, 0.55f, -0.9f)));
	PxRigidActorExt::createExclusiveShape(*leftRoot, PxBoxGeometry(0.5f, 0.05f, 0.05f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*leftRoot, 1.f);

	PxArticulationLink* rightRoot = gArticulation->createLink(base, PxTransform(PxVec3(0.f, 0.55f, 0.9f)));
	PxRigidActorExt::createExclusiveShape(*rightRoot, PxBoxGeometry(0.5f, 0.05f, 0.05f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*rightRoot, 1.f);

	PxArticulationJointReducedCoordinate* joint = static_cast<PxArticulationJointReducedCoordinate*>(leftRoot->getInboundJoint());
	joint->setJointType(PxArticulationJointType::eFIX);
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.25f, -0.9f)));
	joint->setChildPose(PxTransform(PxVec3(0.f, -0.05f, 0.f)));

	//Set up the drive joint...	
	gDriveJoint = static_cast<PxArticulationJointReducedCoordinate*>(rightRoot->getInboundJoint());
	gDriveJoint->setJointType(PxArticulationJointType::ePRISMATIC);
	gDriveJoint->setMotion(PxArticulationAxis::eZ, PxArticulationMotion::eLIMITED);
	gDriveJoint->setLimit(PxArticulationAxis::eZ, -1.4f, 0.2f);
	gDriveJoint->setDrive(PxArticulationAxis::eZ, 100000.f, 0.f, PX_MAX_F32);

	gDriveJoint->setParentPose(PxTransform(PxVec3(0.f, 0.25f, 0.9f)));
	gDriveJoint->setChildPose(PxTransform(PxVec3(0.f, -0.05f, 0.f)));

	const PxU32 linkHeight = 3;
	PxArticulationLink* currLeft = leftRoot, *currRight = rightRoot;

	PxQuat rightParentRot(PxIdentity);
	PxQuat leftParentRot(PxIdentity);
	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const PxVec3 pos(0.5f, 0.55f + 0.1f*(1 + i), 0.f);
		PxArticulationLink* leftLink = gArticulation->createLink(currLeft, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), leftRot));
		PxRigidActorExt::createExclusiveShape(*leftLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*leftLink, 1.f);

		const PxVec3 leftAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), -0.9f);

		joint = static_cast<PxArticulationJointReducedCoordinate*>(leftLink->getInboundJoint());
		joint->setParentPose(PxTransform(currLeft->getGlobalPose().transformInv(leftAnchorLocation), leftParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot));
		joint->setJointType(PxArticulationJointType::eREVOLUTE);

		leftParentRot = leftRot;

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimit(PxArticulationAxis::eTWIST, -PxPi, angle);


		PxArticulationLink* rightLink = gArticulation->createLink(currRight, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), rightRot));
		PxRigidActorExt::createExclusiveShape(*rightLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*rightLink, 1.f);

		const PxVec3 rightAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), 0.9f);

		joint = static_cast<PxArticulationJointReducedCoordinate*>(rightLink->getInboundJoint());
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setParentPose(PxTransform(currRight->getGlobalPose().transformInv(rightAnchorLocation), rightParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot));
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimit(PxArticulationAxis::eTWIST, -angle, PxPi);

		rightParentRot = rightRot;

		PxD6Joint* d6joint = PxD6JointCreate(*gPhysics, leftLink, PxTransform(PxIdentity), rightLink, PxTransform(PxIdentity));

		d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLink;
		currRight = leftLink;
	}


	PxArticulationLink* leftTop = gArticulation->createLink(currLeft, currLeft->getGlobalPose().transform(PxTransform(PxVec3(-0.5f, 0.f, -1.0f), leftParentRot)));
	PxRigidActorExt::createExclusiveShape(*leftTop, PxBoxGeometry(0.5f, 0.05f, 0.05f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*leftTop, 1.f);

	PxArticulationLink* rightTop = gArticulation->createLink(currRight, currRight->getGlobalPose().transform(PxTransform(PxVec3(-0.5f, 0.f, 1.0f), rightParentRot)));
	PxRigidActorExt::createExclusiveShape(*rightTop, PxCapsuleGeometry(0.05f, 0.8f), *gMaterial);
	//PxRigidActorExt::createExclusiveShape(*rightTop, PxBoxGeometry(0.5f, 0.05f, 0.05f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*rightTop, 1.f);

	joint = static_cast<PxArticulationJointReducedCoordinate*>(leftTop->getInboundJoint());
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.f, -1.f), currLeft->getGlobalPose().q.getConjugate()));
	joint->setChildPose(PxTransform(PxVec3(0.5f, 0.f, 0.f), leftTop->getGlobalPose().q.getConjugate()));
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
	//joint->setDrive(PxArticulationAxis::eTWIST, 0.f, 10.f, PX_MAX_F32);

	joint = static_cast<PxArticulationJointReducedCoordinate*>(rightTop->getInboundJoint());
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.f, 1.f), currRight->getGlobalPose().q.getConjugate()));
	joint->setChildPose(PxTransform(PxVec3(0.5f, 0.f, 0.f), rightTop->getGlobalPose().q.getConjugate()));
	joint->setJointType(PxArticulationJointType::eREVOLUTE);
	joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eFREE);
	//joint->setDrive(PxArticulationAxis::eTWIST, 0.f, 10.f, PX_MAX_F32);


	currLeft = leftRoot;
	currRight = rightRoot;

	rightParentRot = PxQuat(PxIdentity);
	leftParentRot = PxQuat(PxIdentity);

	for (PxU32 i = 0; i < linkHeight; ++i)
	{
		const PxVec3 pos(-0.5f, 0.55f + 0.1f*(1 + i), 0.f);
		PxArticulationLink* leftLink = gArticulation->createLink(currLeft, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), leftRot));
		PxRigidActorExt::createExclusiveShape(*leftLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*leftLink, 1.f);

		const PxVec3 leftAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), -0.9f);

		joint = static_cast<PxArticulationJointReducedCoordinate*>(leftLink->getInboundJoint());
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setParentPose(PxTransform(currLeft->getGlobalPose().transformInv(leftAnchorLocation), leftParentRot));
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, -1.f), rightRot));

		leftParentRot = leftRot;

		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimit(PxArticulationAxis::eTWIST, -PxPi, angle);

		PxArticulationLink* rightLink = gArticulation->createLink(currRight, PxTransform(pos + PxVec3(0.f, sinAng*(2 * i + 1), 0.f), rightRot));
		PxRigidActorExt::createExclusiveShape(*rightLink, PxBoxGeometry(0.05f, 0.05f, 1.f), *gMaterial);
		PxRigidBodyExt::updateMassAndInertia(*rightLink, 1.f);

		const PxVec3 rightAnchorLocation = pos + PxVec3(0.f, sinAng*(2 * i), 0.9f);

		/*joint = PxD6JointCreate(getPhysics(), currRight, PxTransform(currRight->getGlobalPose().transformInv(rightAnchorLocation)),
		rightLink, PxTransform(PxVec3(0.f, 0.f, 1.f)));*/

		joint = static_cast<PxArticulationJointReducedCoordinate*>(rightLink->getInboundJoint());
		joint->setParentPose(PxTransform(currRight->getGlobalPose().transformInv(rightAnchorLocation), rightParentRot));
		joint->setJointType(PxArticulationJointType::eREVOLUTE);
		joint->setChildPose(PxTransform(PxVec3(0.f, 0.f, 1.f), leftRot));
		joint->setMotion(PxArticulationAxis::eTWIST, PxArticulationMotion::eLIMITED);
		joint->setLimit(PxArticulationAxis::eTWIST, -angle, PxPi);

		rightParentRot = rightRot;

		PxD6Joint* d6joint = PxD6JointCreate(*gPhysics, leftLink, PxTransform(PxIdentity), rightLink, PxTransform(PxIdentity));

		d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
		d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

		currLeft = rightLink;
		currRight = leftLink;
	}

	PxD6Joint* d6joint = PxD6JointCreate(*gPhysics, currLeft, PxTransform(PxVec3(0.f, 0.f, -1.f)), leftTop, PxTransform(PxVec3(-0.5f, 0.f, 0.f)));

	d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);

	d6joint = PxD6JointCreate(*gPhysics, currRight, PxTransform(PxVec3(0.f, 0.f, 1.f)), rightTop, PxTransform(PxVec3(-0.5f, 0.f, 0.f)));

	d6joint->setMotion(PxD6Axis::eTWIST, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING1, PxD6Motion::eFREE);
	d6joint->setMotion(PxD6Axis::eSWING2, PxD6Motion::eFREE);


	const PxTransform topPose(PxVec3(0.f, leftTop->getGlobalPose().p.y + 0.15f, 0.f));

	PxArticulationLink* top = gArticulation->createLink(leftTop, topPose);
	PxRigidActorExt::createExclusiveShape(*top, PxBoxGeometry(0.5f, 0.1f, 1.5f), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*top, 1.f);

	joint = static_cast<PxArticulationJointReducedCoordinate*>(top->getInboundJoint());
	joint->setJointType(PxArticulationJointType::eFIX);
	joint->setParentPose(PxTransform(PxVec3(0.f, 0.0f, 0.f)));
	joint->setChildPose(PxTransform(PxVec3(0.f, -0.15f, -0.9f)));

	gScene->addArticulation(*gArticulation);

	for (PxU32 i = 0; i < gArticulation->getNbLinks(); ++i)
	{
		PxArticulationLink* link;
		gArticulation->getLinks(&link, 1, i);

		link->setLinearDamping(0.2f);
		link->setAngularDamping(0.2f);

		link->setMaxAngularVelocity(20.f);
		link->setMaxLinearVelocity(100.f);

		if (link != top)
		{
			for (PxU32 b = 0; b < link->getNbShapes(); ++b)
			{
				PxShape* shape;
				link->getShapes(&shape, 1, b);

				shape->setSimulationFilterData(PxFilterData(0, 0, 1, 0));
			}
		}
	}

	const PxVec3 halfExt(0.25f);
	const PxReal density(0.5f);

/*	PxRigidDynamic* box0 = gPhysics->createRigidDynamic(PxTransform(PxVec3(-0.25f, 5.f, 0.5f)));
	PxRigidActorExt::createExclusiveShape(*box0, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box0, density);

	gScene->addActor(*box0);

	PxRigidDynamic* box1 = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.25f, 5.f, 0.5f)));
	PxRigidActorExt::createExclusiveShape(*box1, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box1, density);

	gScene->addActor(*box1);

	PxRigidDynamic* box2 = gPhysics->createRigidDynamic(PxTransform(PxVec3(-0.25f, 4.5f, 0.5f)));
	PxRigidActorExt::createExclusiveShape(*box2, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box2, density);

	gScene->addActor(*box2);

	PxRigidDynamic* box3 = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.25f, 4.5f, 0.5f)));
	PxRigidActorExt::createExclusiveShape(*box3, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box3, density);

	gScene->addActor(*box3);

	PxRigidDynamic* box4 = gPhysics->createRigidDynamic(PxTransform(PxVec3(-0.25f, 5.f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*box4, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box4, density);

	gScene->addActor(*box4);

	PxRigidDynamic* box5 = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.25f, 5.f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*box5, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box5, density);

	gScene->addActor(*box5);

	PxRigidDynamic* box6 = gPhysics->createRigidDynamic(PxTransform(PxVec3(-0.25f, 4.5f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*box6, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box6, density);

	gScene->addActor(*box6);

	PxRigidDynamic* box7 = gPhysics->createRigidDynamic(PxTransform(PxVec3(0.25f, 4.5f, 0.f)));
	PxRigidActorExt::createExclusiveShape(*box7, PxBoxGeometry(halfExt), *gMaterial);
	PxRigidBodyExt::updateMassAndInertia(*box7, density);

	gScene->addActor(*box7);*/
}
