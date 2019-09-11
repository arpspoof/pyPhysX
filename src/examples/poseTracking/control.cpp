#include "config.h"
#include "Articulation.h"
#include "PxPhysicsAPI.h"

#include <Eigen/Dense>

using namespace physx;
using namespace Eigen;
using namespace std;

extern Articulation* articulation;

/*

2 chest			0, 1, 2
3 right_hip		3, 4, 5
4 left_hip		6, 7, 8
5 neck			9, 10, 11
6 right_shoulder	12, 13, 14
7 left_shoulder	15, 16, 17	
8 right_knee		18	
9 left_knee		19
10 right_elbow		20
11 left_elbow		21
12 right_ankle		22
13 left_ankle		23

*/

// jointPosition is twist + swing exp map [qSwing * qTwist]

extern PxReal motions[98][36];

int idMap[] = { 0, 8, 22, 4, 17, 31, 12, 26, 21, 35, 13, 27 };

void InitControl() {
	vector<float> kps, kds, fls;
	int totalDof = articulation->GetNDof();

	kps = vector<float>(totalDof);
	kds = vector<float>(totalDof);
	fls = vector<float>(totalDof);

	for (auto &kvp : articulation->jointMap) {
		auto &joint = kvp.second;
		const string &name = kvp.first;

		int nDof = joint->nDof;
		int cacheIndex = joint->cacheIndex;

		for (int i = 0; i < nDof; i++) {
			kps[cacheIndex + i] = getConfigF("P_KP_" + name) * 10;
			kds[cacheIndex + i] = getConfigF("P_KD_" + name) * 1;
			fls[cacheIndex + i] = getConfigF("P_FL_" + name);
		}
	}

	articulation->SetKPs(kps.data());
	articulation->SetKDs(kds.data());
}

extern int xFrame;

float targetPosition[36];

void control(PxReal dt) {
	int index = 0;
	const int* dofs = articulation->GetJointDofsInIdOrder();
	for (int i = 0; i < 12; i++) {
		int dof = dofs[i];
		int dataid = idMap[i];
		int loopcount = dof == 3 ? 4 : 1;
		for (int j = 0; j < loopcount; j++) {
			targetPosition[index + j] = motions[xFrame][dataid + j];
		}
		index += loopcount;
	}
	articulation->AddSPDForces(targetPosition, dt);
}
