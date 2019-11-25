
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


#include <iomanip>
#include <filesystem>
string basePath = "/home/zhiqiy/Documents/motionsFixed/";
string outPath = "/home/zhiqiy/Documents/motionsFixed2/";

void runTransform() 
{
    vector<string> linkShapes {
        "s", "s", "s", 
        "c", "c", "b", "c", "c", "s", 
        "c", "c", "b", "c", "c", "s", 
    };
    vector<PxVec3> linkParams {
        PxVec3(0.36f), PxVec3(0.44f), PxVec3(0.41f),
        PxVec3(0.22f, 1.2f, 0.22f), PxVec3(0.2f, 1.24f, 0.2f), PxVec3(0.708f, 0.22f, 0.36f), 
        PxVec3(0.18f, 0.72f, 0.18f), PxVec3(0.16f, 0.54f, 0.16f), PxVec3(0.16f), 
        PxVec3(0.22f, 1.2f, 0.22f), PxVec3(0.2f, 1.24f, 0.2f), PxVec3(0.708f, 0.22f, 0.36f), 
        PxVec3(0.18f, 0.72f, 0.18f), PxVec3(0.16f, 0.54f, 0.16f), PxVec3(0.16f)
    };
    for (PxVec3& v : linkParams) {
        v *= 0.25f;
    }

    for (const auto & entry : filesystem::directory_iterator(basePath)) {
        string path = entry.path();
        printf("processing %s\n", path.c_str());
        
        ifstream motioninput(path);
        string motionStr((istreambuf_iterator<char>(motioninput)), istreambuf_iterator<char>());
        
        auto motionJson = json::parse(motionStr);
        auto& frames = motionJson["Frames"];

        for (auto& frame : frames) {
            vector<float> jp;
            for (int i = 0; i < 43; i++) jp.push_back(frame[i + 1]);
            articulation->SetJointPositionsQuaternion(jp);
            articulation->CalculateFK(jp);

            float lowest = 1000;
            for (int i = 0; i < 15; i++) {
                PxVec3 pos = articulation->linkPositions[i];
                PxQuat rot = articulation->linkGlobalRotations[i];
                if (linkShapes[i] == "s") {
                    float y = pos.y - linkParams[i].y;
                    lowest = min(lowest, y);
                }
                else if (linkShapes[i] == "b") {
                    for (int j = 0; j < 8; j++) {
                        int sign1 = (j & 1) ? 1 : -1;
                        int sign2 = (j & 2) ? 1 : -1;
                        int sign3 = (j & 4) ? 1 : -1;
                        PxVec3 localOffset = linkParams[i] * 0.5f;
                        localOffset.x *= sign1;
                        localOffset.y *= sign2;
                        localOffset.z *= sign3;
                        PxVec3 globalOffset = rot.rotate(localOffset);
                        lowest = min(lowest, globalOffset.y + pos.y);
                    }
                }
                else {
                    for (int j = 0; j < 2; j++) {
                        int sign1 = (j & 1) ? 1 : -1;
                        PxVec3 localOffset(0, linkParams[i].y * sign1 * 0.5f, 0);
                        PxVec3 globalOffset = rot.rotate(localOffset);
                        lowest = min(lowest, globalOffset.y + pos.y - linkParams[i].x);
                    }
                }
            }

            if (lowest < 0) {
                printf("lowest = %f\n", lowest);
                jp[1] += -lowest + 0.0001f;
                frame[2] = jp[1];
            } 
            else {
                printf(".");
            }
        }

        string name = path.substr(path.find_last_of('/') + 1);
        ofstream out(outPath + name);
        out << setw(4) << motionJson << endl;
        out.close();
        printf("output to %s\n", (outPath + name).c_str());
    }
}


























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
        ("m,mocap", "Mocap data path", cxxopts::value<string>()->default_value("run.txt"))
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


    runTransform();
    return 0;


 /*   if (result["performance"].as<bool>()) {
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

    return 0;*/
}
