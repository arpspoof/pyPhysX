#include "config.h"
#include "Articulation.h"
#include "PxPhysicsAPI.h"

using namespace physx;
using namespace std;

extern Articulation* articulation;
extern PxReal motions[98][36];

void InitControl() {
    vector<float> kps, kds, fls;

    for (auto &j : articulation->GetAllJointsInIdOrder()) {
        const string &name = j->name;
        int nDof = j->nDof;

        for (int i = 0; i < nDof; i++) {
            kps.push_back(getConfigF("P_KP_" + name) * 10);
            kds.push_back(getConfigF("P_KD_" + name) * 2);
            fls.push_back(getConfigF("P_FL_" + name));
        }
    }

    articulation->SetKPs(kps);
    articulation->SetKDs(kds);

    float rootKpL = 10000.f, rootKdL = 1000.f;
    float rootKpA = 10000.f, rootKdA = 1000.f;
    articulation->root_kps = { rootKpL, rootKpL, rootKpL, rootKpA, rootKpA, rootKpA };
    articulation->root_kds = { rootKdL, rootKdL, rootKdL, rootKdA, rootKdA, rootKdA };
}

extern int xFrame;

void control(PxReal dt) {
    vector<float> targetPosition(36 + 7);
    targetPosition[1] = 4;
    targetPosition[3] = 1;
    for (int i = 0; i < 36; i++) {
        targetPosition[i + 7] = motions[xFrame][i];
    }
    articulation->AddSPDForcesSparse(targetPosition, dt, true);

  /*  auto ps = articulation->GetJointPositionsQuaternion();
    auto vs = articulation->GetJointVelocitiesPack4();

    printf("positions:\n");
    for (float x : ps) {
        printf("%f, ", x);
    }
    printf("\n");

    printf("velocities:\n");
    for (float x : vs) {
        printf("%f, ", x);
    }
    printf("\n");*/
}
