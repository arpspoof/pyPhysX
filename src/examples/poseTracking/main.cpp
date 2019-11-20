
#include <ctype.h>
#include <vector>
#include <stdio.h>
#include <chrono>

#include "PxPhysicsAPI.h"
#include "ArticulationTree.h"

#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

#include "GlutRenderer.h"
#include "MathInterface.h"
#include "PrimitiveObjects.h"
#include "Foundation.h"
#include "UrdfLoader.h"
#include "JsonLoader.h"
#include "Scene.h"
#include "Actor.h"
#include "cxxopts.hpp"
#include "json.hpp"

using namespace physx;
using namespace std;
using namespace std::chrono;
using namespace nlohmann;

static Foundation* foundation;
static Articulation* articulation;
static Scene* scene;
static Material* material;

static float kp, kd, rkpA, rkdA, rkpL, rkdL;
static vector<vector<float>> motions;
static int xFrame = 0;
static int nRounds = 0;

extern bool debug;
extern bool dump;

static bool useDog;

static int dim;

void reset()
{
    dim = 7;
    auto dofs = articulation->GetJointDofsInIdOrder();

    vector<float> p { 0, 3.55f, 0, 1, 0, 0, 0 };
    for (int d : dofs) {
        if (d == 1) {
            dim++;
            p.push_back(0);
        }
        else if (d == 3) {
            dim += 4;
            p.push_back(1);
            p.push_back(0);
            p.push_back(0);
            p.push_back(0);
        }
    }

    if (dim == 95) {
        // dog
        p[1] = 2.781f;
        float r2 = 0.70710678118f;
        int legs[4] = { 23, 39, 55, 71 };
        for (int i = 0; i < 4; i++) {
            p[legs[i]] = r2;
            p[legs[i] + 3] = -r2;
        }
    }

    vector<float> v(dim, 0);
    articulation->SetJointVelocitiesPack4(v);
    articulation->SetJointPositionsQuaternion(p);
}

void InitControl() {
    vector<float> kps, kds, fls;

    for (auto &j : articulation->GetAllJointsInIdOrder()) {
        int nDof = j->nDof;

        for (int i = 0; i < nDof; i++) {
            kps.push_back(kp);
            kds.push_back(kd);
        }
    }

    articulation->SetKPs(kps);
    articulation->SetKDs(kds);

    float rootKpL = rkpL, rootKdL = rkdL;
    float rootKpA = rkpA, rootKdA = rkdA;
    // 2000, 1600 is a good combo for walking (t = 0.033, kp = 75000, kd = 6600)
    articulation->root_kps = { rootKpL, rootKpL, rootKpL, rootKpA, rootKpA, rootKpA };
    articulation->root_kds = { rootKdL, rootKdL, rootKdL, rootKdA, rootKdA, rootKdA };

//    reset();
}

void initPhysics(float dt)
{
    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), dt);

    material = scene->CreateMaterial(1.f, 1.f, 0.f);

    auto plane = scene->CreatePlane(material, vec3(0, 1, 0), 0);
    plane->SetupCollisionFiltering(1, -1);

    if (useDog) {
        JsonLoader jsonLoader(4);
        jsonLoader.LoadDescriptionFromFile("resources/dog3d.txt");
        articulation = scene->CreateArticulation(&jsonLoader, material, vec3(0, 3.25f, 0));
    }
    else {
        articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 2.781f, 0), 0.25f);
    }

    InitControl();

    auto alljoints = articulation->GetAllJointsInIdOrder();
    for (auto j : alljoints) {
        j->childLink->SetupCollisionFiltering(2 << (j->id), -1);
        printf("group for %s is %d\n", j->name.c_str(), 2 << (j->id));
    }
}
    
void cleanupPhysics()
{
    foundation->Dispose();
    delete foundation;
}

static int controller = 0; // 0-ABA 1-Sparse 2-Dense
static float trackingFrequency = 0.033f;
static float height;

static ofstream ot;
static ofstream oc;

void control(PxReal dt) {
    static int currentRound = 0;
    static float cumulateTime = 0;

    auto motionFrame = motions[xFrame];
    motionFrame[0] += motions.back()[0] * currentRound;
    motionFrame[2] += motions.back()[2] * currentRound;

    switch (controller)
    {
    case 0:
        articulation->AddSPDForcesABA(motionFrame, dt, true);
        break;
    case 1:
        articulation->AddSPDForcesSparse(motionFrame, dt, true);
        break;
    case 2:
        articulation->AddSPDForces(motionFrame, dt, true);
        break;
    case 3:
        // Kinematic controller still has artifacts for cartwheel.
        // Maybe still due to backend bug.
        articulation->SetJointPositionsQuaternion(motionFrame);
        articulation->SetJointVelocitiesPack4(vector<float>(dim, 0));
        break;
    case 4:
        articulation->AddSPDForcesAcc(motionFrame, dt);
        break;
    default:
        break;
    }

    if (dump) {
        for (int i = 0; i < dim; i++) ot << motionFrame[i] << " ";
        ot << endl;

        auto current = articulation->GetJointPositionsQuaternion();
        for (int i = 0; i < dim; i++) oc << current[i] << " ";
        oc << endl;
    }

    cumulateTime += dt;
    if (cumulateTime >= (currentRound * motions.size() + xFrame + 1) * trackingFrequency) {
        xFrame = (xFrame + 1) % motions.size();
        if (xFrame == 0) {
            currentRound++;
            if (nRounds > 0 && currentRound == nRounds) {
                reset();
                currentRound = 0;
                cumulateTime = 0;
            }
        }
    }

/*    auto jp = articulation->GetJointPositionsQuaternion();
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
    printf("-----------------------------------------------------------\n");*/
}

class GlutHandler :public glutRenderer::GlutRendererCallback
{
    void keyboardHandler(unsigned char key) override
    {
        switch (key) {
        case 'z': xFrame--; printf("xframe = %d\n", xFrame); break;
        case 'x': xFrame++; printf("xframe = %d\n", xFrame); break;
        }
    }
    void beforeSimulationHandler() override
    {
        control(scene->timeStep);
    }
} glutHandler;

vector<float> setpos {
    1.53139758,  0.76897873,  0.,          0.97438135,  0.07631286,  0.03776851,
 -0.20816072,  0.99517433, -0.08416767,  0.00331635, -0.05032751,  1.,
  0.,          0.,          0.,          0.96076651, -0.10451415, -0.07597877,
  0.24542148, -1.72684866,  0.98388587,  0.17281356, -0.04579028, -0.00271015,
  0.93162382, -0.25565315,  0.24258843, -0.08870958,  1.00557206,  0.89450882,
 -0.09429659, -0.04680736,  0.43447806, -0.92785196,  0.96074693,  0.01482465,
 -0.00889803,  0.27688699,  0.94699727,  0.25834359, -0.18217961,  0.0571433,
  1.3976368
};

vector<float> setvel {
    2.83644298,  0.14862616,  0.,          0.25620783,  0.88736969, -1.21504141,
  0.,          0.81095874, -0.41069279, -0.05612331,  0.,          0.,
  0.,          0.,          0.,          0.11502411, -0.37685434,  9.63703945,
  0.,         -2.26660043,  0.74523553, -0.31609161,  2.85611727,  0.,
  1.76529462,  4.96363324, -4.7674136,   0.,         -4.75561945, -0.48500878,
 -0.49944053, -2.74588429,  0.,          0.57918268,  0.90546744, -0.65763673,
  3.1667057,   0.,          3.26858449,  0.59680633,  9.85829134,  0.,
  2.30806795
};

vector<float> setspd {
    0.9954484105110168, -0.07091088593006134, -0.06349769979715347, -0.004711734596639872, 0.9968694448471069, -0.058582376688718796, 0.029735218733549118, 0.04399218037724495, 0.9144092202186584, -0.1510108858346939, -0.05221044644713402, 0.37192144989967346, -2.0294315814971924, 0.967540979385376, 0.24877884984016418, -0.03705485537648201, -0.024506621062755585, 0.8977230191230774, -0.25066307187080383, 0.293827623128891, -0.21195927262306213, 0.9495632648468018, 0.8612546920776367, -0.13943254947662354, -0.09141234308481216, 0.48004451394081116, -1.1590757369995117, 0.9470224976539612, 0.010832543484866619, -0.029360629618167877, 0.3196389079093933, 0.8936602473258972, 0.31390172243118286, -0.29510733485221863, 0.1254938393831253, 1.4383859634399414
};

int main(int argc, char** argv)
{
    cxxopts::Options opts("Example", "Pose tracking");
    opts.add_options()
        ("d,dog", "Use dog model")
        ("p,performance", "Run performance test")
        ("dbg", "Print debug info")
        ("dmp", "Dump simulation target and actuals")
        ("r,round", "Number of rounds", cxxopts::value<int>()->default_value("0"))
        ("c,control", "Controller", cxxopts::value<int>()->default_value("0"))
        ("m,mocap", "Mocap data path", cxxopts::value<string>()->default_value("testMotion.txt"))
        ("f,frequency", "Tracking frequency", cxxopts::value<float>()->default_value("0.033"))
        ("t,dt", "Time step", cxxopts::value<float>()->default_value("0.033"))
        ("kp", "Joint kp", cxxopts::value<float>()->default_value("75000"))
        ("kd", "Joint kd", cxxopts::value<float>()->default_value("6600"))
        ("rkpa", "Root kp angular", cxxopts::value<float>()->default_value("2000"))
        ("rkda", "Root kd angular", cxxopts::value<float>()->default_value("1800"))
        ("rkpl", "Root kp linear", cxxopts::value<float>()->default_value("2000"))
        ("rkdl", "Root kd linear", cxxopts::value<float>()->default_value("100"))
        ("h,height", "height", cxxopts::value<float>()->default_value("0"));
    
    auto result = opts.parse(argc, argv);

    debug = result["dbg"].as<bool>();
    dump = result["dmp"].as<bool>();
    useDog = result["dog"].as<bool>();

    kp = result["kp"].as<float>();
    kd = result["kd"].as<float>();
    rkpA = result["rkpa"].as<float>();
    rkdA = result["rkda"].as<float>();
    rkpL = result["rkpl"].as<float>();
    rkdL = result["rkdl"].as<float>();
    nRounds = result["round"].as<int>();
    controller = result["control"].as<int>();
    trackingFrequency = result["frequency"].as<float>();
    height = result["height"].as<float>();

    if (dump) {
        ot.open("/home/zhiqiy/target.txt");
        oc.open("/home/zhiqiy/current.txt");
    }

    initPhysics(result["dt"].as<float>());

    string mocap = result["mocap"].as<string>();
    printf("mocap is %s\n", mocap.c_str());

    ifstream motioninput("./resources/motions/" + mocap);
    string motionStr((istreambuf_iterator<char>(motioninput)), istreambuf_iterator<char>());
    
    auto motionJson = json::parse(motionStr);
    auto frames = motionJson["Frames"];

    if (useDog) {
        // motion retargeting
        for (auto frame : frames) {
            motions.push_back(vector<float>(dim));
            auto& motion = motions.back();
            int offset = 0;
            for (int i = 0; i < 83; i++) {
                float value = frame[i + 1]; 
                if (i <= 2) value *= 4;
                if (i == 1) value += height;
                if (i == 27 || i == 40 || i == 53 || i == 66) {
                    PxQuat q(value, PxVec3(0, 0, 1));
                    motion[i + offset] = q.w;
                    motion[i + offset + 1] = q.x;
                    motion[i + offset + 2] = q.y;
                    motion[i + offset + 3] = q.z;
                    offset += 3;
                }
                motion[i + offset] = value;
            }
            int legs[4] = { 23, 39, 55, 71 };
            for (int i = 0; i < 4; i++) {
                int leg = legs[i];
                PxQuat qOriginal(motion[leg + 1], motion[leg + 2], motion[leg + 3], motion[leg]);
                PxQuat qRetarget = qOriginal * PxQuat(-PxPi / 2, PxVec3(0, 0, 1));
                motion[leg] = qRetarget.w;
                motion[leg + 1] = qRetarget.x;
                motion[leg + 2] = qRetarget.y;
                motion[leg + 3] = qRetarget.z;
            }
            int retargetId[] = { 6, 7, 8, 10, 11, 12, 14, 15, 16, 18, 19, 20 };
            for (int id : retargetId) {
                int cacheIndex = 7 + (id - 1) * 4;
                PxQuat qOriginal(motion[cacheIndex + 1], motion[cacheIndex + 2], 
                    motion[cacheIndex + 3], motion[cacheIndex]);
                PxQuat frameTransform(-PxPi / 2, PxVec3(0, 0, 1));
                PxQuat qRetarget = frameTransform.getConjugate() * qOriginal * frameTransform;
                motion[cacheIndex] = qRetarget.w;
                motion[cacheIndex + 1] = qRetarget.x;
                motion[cacheIndex + 2] = qRetarget.y;
                motion[cacheIndex + 3] = qRetarget.z;
            }
        }
    }
    else {
        for (auto frame : frames) {
            motions.push_back(vector<float>(dim));
            for (int i = 0; i < dim; i++) {
                float value = frame[i + 1]; 
                if (i <= 2) value *= 4;
                if (i == 1) value += height;
                motions.back()[i] = value;
            }
        }
    }

    motioninput.close();
    printf("%ld lines\n", motions.size());

//    articulation->SetJointPositionsQuaternion(motions[0]);

    articulation->SetJointPositionsQuaternion(setpos);
    articulation->SetJointVelocitiesPack4(setvel);
    scene->timeStep = 0.00001f;
    scene->Step();
    auto contact = scene->GetAllContactPairs();
    for (auto c : contact) {
        printf("contact: %d, %d\n", c.first, c.second);
    }
    printf("................\n");

    scene->timeStep = 0.0016666666666666666f;
    articulation->AddSPDForces(setspd, scene->timeStep);
    scene->Step();
    contact = scene->GetAllContactPairs();
    for (auto c : contact) {
        printf("contact: %d, %d\n", c.first, c.second);
    }
/*    auto jp = articulation->GetJointPositionsQuaternion();
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
    printf("-----------------------------------------------------------\n");*/

    if (result["performance"].as<bool>()) {
        static const PxU32 frameCount = 10000;
        auto starttime = high_resolution_clock::now();
        for(PxU32 i = 0; i < frameCount; i++) {
            control(scene->timeStep);
            scene->Step();
        }
        auto endtime = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(endtime - starttime).count();
        printf("%ld\n", duration);
    }
    else {
        auto renderer = glutRenderer::GlutRenderer::GetInstance();
        renderer->AttachScene(scene, &glutHandler);
        renderer->StartRenderLoop();
        cleanupPhysics();
    }

    if (dump) {
        ot.close();
        oc.close();
    }

    return 0;
}
