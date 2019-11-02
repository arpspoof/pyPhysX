
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
#include "Scene.h"
#include "Actor.h"
#include "cxxopts.hpp"

using namespace physx;
using namespace std;
using namespace std::chrono;

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

void reset()
{
    vector<float> p { 0, 3.55f, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0};
    vector<float> v(43, 0);
    articulation->SetJointVelocitiesPack4(v);
    articulation->SetJointPositionsQuaternion(p);
}

vector<string> collisionGroupNames {
    "floor", "root", "chest", "neck",
    "left_hip", "left_knee", "left_ankle",
    "right_hip", "right_knee", "right_ankle",
    "left_shoulder", "left_elbow", "left_wrist",
    "right_shoulder", "right_elbow", "right_wrist"
};

static int GetFirstOne(int x) 
{
    if (x == 0) return -1;
    int r = 0;
    while (!(x & 1)) { r++; x >>= 1; }
    return r;
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

    articulation->linkMap["root"]->SetupCollisionFiltering(1 << 1, -1);
    articulation->linkMap["chest"]->SetupCollisionFiltering(1 << 2, -1);
    articulation->linkMap["neck"]->SetupCollisionFiltering(1 << 3, -1);
    articulation->linkMap["right_hip"]->SetupCollisionFiltering(1 << 4, -1);
    articulation->linkMap["right_knee"]->SetupCollisionFiltering(1 << 5, -1);
//    articulation->linkMap["right_ankle"]->SetupCollisionFiltering(1 << 6, -1);
    articulation->linkMap["right_shoulder"]->SetupCollisionFiltering(1 << 7, -1);
    articulation->linkMap["right_elbow"]->SetupCollisionFiltering(1 << 8, -1);
//    articulation->linkMap["right_wrist"]->SetupCollisionFiltering(1 << 9, -1);
    articulation->linkMap["left_hip"]->SetupCollisionFiltering(1 << 10, -1);
    articulation->linkMap["left_knee"]->SetupCollisionFiltering(1 << 11, -1);
//    articulation->linkMap["left_ankle"]->SetupCollisionFiltering(1 << 12, -1);
    articulation->linkMap["left_shoulder"]->SetupCollisionFiltering(1 << 13, -1);
    articulation->linkMap["left_elbow"]->SetupCollisionFiltering(1 << 14, -1);
//    articulation->linkMap["left_wrist"]->SetupCollisionFiltering(1 << 15, -1);

    reset();
}

void initPhysics(float dt)
{
    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), dt);

    material = scene->CreateMaterial(1.f, 1.f, 0.f);

    auto plane = scene->CreatePlane(material, vec3(0, 1, 0), 0);
    plane->SetupCollisionFiltering(1 << 0, -1);
    
    articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 3.75f, 0));

    InitControl();
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
        articulation->SetJointPositionsQuaternion(motionFrame);
        articulation->SetJointVelocitiesPack4(vector<float>(43, 0));
        break;
    default:
        break;
    }

    if (dump) {
        for (int i = 0; i < 43; i++) ot << motionFrame[i] << " ";
        ot << endl;

        auto current = articulation->GetJointPositionsQuaternion();
        for (int i = 0; i < 43; i++) oc << current[i] << " ";
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
        auto contacts = scene->GetAllContactPairs();
        for (auto c : contacts) {
            printf("contact: %s with %s\n", collisionGroupNames[GetFirstOne(c.first)].c_str(), 
                collisionGroupNames[GetFirstOne(c.second)].c_str());
        }
        control(scene->timeStep);
    }
} glutHandler;

int main(int argc, char** argv)
{
    cxxopts::Options opts("Example", "Pose tracking");
    opts.add_options()
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

    string mocap = result["mocap"].as<string>();
    printf("mocap is %s\n", mocap.c_str());

    ifstream motioninput("../resources/motions/" + mocap);

    float tmp; int col = 0;
    while (motioninput >> tmp) {
        if (col == 0) motions.push_back(vector<float>(43));
        if (col <= 2) tmp *= 4;
        if (col == 1) tmp += height;
        motions.back()[col] = tmp;
        col = (col + 1) % 43;
    }
    motioninput.close();
    printf("%ld lines\n", motions.size());

    initPhysics(result["dt"].as<float>());

    // TODO: backflip force direction issue? What's wrong with set quat?
 /*   articulation->SetJointPositionsQuaternion(motions[3]);
    auto test = articulation->GetJointPositionsQuaternion();

    printf("origin\n");
    for (int i = 0; i < 43; i++) printf("%f, ", motions[3][i]);
    printf("\n");
    printf("conver\n");
    for (int i = 0; i < 43; i++) printf("%f, ", test[i]);
    printf("\n");*/

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
