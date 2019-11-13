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
            kds.push_back(getConfigF("P_KD_" + name) * 1);
            fls.push_back(getConfigF("P_FL_" + name));
        }
    }

    articulation->SetKPs(kps);
    articulation->SetKDs(kds);
}

extern int xFrame;

void control(PxReal dt) {
    vector<float> targetPosition(36);
    for (int i = 0; i < 36; i++) {
        targetPosition[i] = motions[xFrame][i];
    }
    articulation->AddSPDForces(targetPosition, dt);
    

 /*   auto ps = articulation->GetJointPositionsQuaternion();
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

    auto jp = articulation->GetJointPositionsQuaternion();
    articulation->CalculateFK(jp);
    auto alljoints = articulation->GetAllJointsInIdOrder();
    for (auto j : alljoints) {
        printf("link name = %s\n", j->name.c_str());
        PxVec3 pos = articulation->linkPositions[j->id];
        PxQuat rot = articulation->linkGlobalRotations[j->id];
        auto link = articulation->GetLinkByName(j->name);
        auto linkTransform = link->link->getGlobalPose();
        PxQuat frameTransform(-PxPi / 2, PxVec3(0, 0, 1));
        PxVec3 pxpos = linkTransform.p;
        PxQuat pxrot = linkTransform.q * frameTransform;
        printf("my transform: p = %f, %f, %f; q = %f, %f, %f, %f\n",
            pos.x, pos.y, pos.z, rot.w, rot.x, rot.y, rot.z);
        printf("px transform: p = %f, %f, %f; q = %f, %f, %f, %f\n",
            pxpos.x, pxpos.y, pxpos.z, pxrot.w, pxrot.x, pxrot.y, pxrot.z);
    }
}
