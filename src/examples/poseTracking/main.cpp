
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

#include "UnityRenderer.h"
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

static long spdTime = 0;

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

    reset();
}

void initPhysics(float dt)
{
    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), dt);

    material = scene->CreateMaterial(1.f, 1.f, 0.f);

    auto plane = scene->CreatePlane(material, vec3(0, 1, 0), 0);
    plane->SetupCollisionFiltering(1, 2 | 4);

    if (useDog) {
        JsonLoader jsonLoader(4);
        jsonLoader.LoadDescriptionFromFile("resources/dog3d_my.txt");
        articulation = scene->CreateArticulation(&jsonLoader, material, vec3(0, 3.25f, 0));
    }
    else {
        articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 2.781f, 0));
    }

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
    
    auto starttime = high_resolution_clock::now();
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
    default:
        break;
    }
    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    spdTime += duration;

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

    for (auto frame : frames) {
        motions.push_back(vector<float>(dim));
        for (int i = 0; i < dim; i++) {
            float value = frame[i + 1]; 
            if (i <= 2) value *= 4;
            if (i == 1) value += height;
            motions.back()[i] = value;
        }
    }

    motioninput.close();
    printf("%ld lines\n", motions.size());

    articulation->SetJointPositionsQuaternion(motions[0]);

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
        extern long g_ABA_Timer;
        printf("%ld\n", g_ABA_Timer);
        printf("%ld\n", spdTime);
    }
    else {
    /*    auto renderer = glutRenderer::GlutRenderer::GetInstance();
        renderer->AttachScene(scene, &glutHandler);
        renderer->StartRenderLoop();*/
        UR_Init(scene->timeStep);
        UR_AddArticulation(articulation);
        UR_InitPrimitives();
        for(;;) {
            control(scene->timeStep);
            scene->Step();
            UR_Tick();
        }
        cleanupPhysics();
    }

    if (dump) {
        ot.close();
        oc.close();
    }

    return 0;
}
