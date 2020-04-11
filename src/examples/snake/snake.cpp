#include "PxPhysicsAPI.h"
#include "UnityRenderer.h"
#include "JsonLoader.h"
#include "Foundation.h"
#include "Scene.h"
#include "cxxopts.hpp"
#include "json.hpp"

#include <string>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>

using namespace nlohmann;
using namespace physx;
using namespace std;
using namespace std::chrono;

static Foundation* foundation;
static Articulation* articulation;
static Scene* scene;
static Material* material;

static bool longSnake = false;
static float height = 0.251f;

void InitControl() {
    vector<float> kps, kds, fls;

    for (auto &j : articulation->GetAllJointsInIdOrder()) {
        int nDof = j->nDof;

        for (int i = 0; i < nDof; i++) {
            kps.push_back(2000);
            kds.push_back(200);
        }
    }

    articulation->SetKPs(kps);
    articulation->SetKDs(kds);

    float rootKpL = 20000, rootKdL = 2000;
    float rootKpA = 20000, rootKdA = 2000;
    // 2000, 1600 is a good combo for walking (t = 0.033, kp = 75000, kd = 6600)
    articulation->root_kps = { rootKpL, rootKpL, rootKpL, rootKpA, rootKpA, rootKpA };
    articulation->root_kds = { rootKdL, rootKdL, rootKdL, rootKdA, rootKdA, rootKdA };
}

void initPhysics(float dt)
{
    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), dt);

    material = scene->CreateMaterial(.5f, .5f, 0.f);

    scene->CreatePlane(material, vec3(0, 1, 0), 0);

    JsonLoader jsonLoader(0.5f);
    string filepath = "resources/snake.txt";
    if (longSnake) filepath = "resources/snake-long.txt";

    jsonLoader.LoadDescriptionFromFile(filepath);
    articulation = scene->CreateArticulation(&jsonLoader, material, vec3(0, height, 0));

    InitControl();
}
    
void cleanupPhysics()
{
    foundation->Dispose();
    delete foundation;
}

static long spdTime = 0;
static int controlMethod = 0;

static float frequencyHz = 30;
static float omega = 0.8;
static float amplitude = 1.3;
static float segdist = 0.65f;
static float pos = 0;

float getsin(float x)
{
    return (float)(amplitude * sin(omega * x));
}

float getcos(float x)
{
    return (float)(omega * amplitude * cos(omega * x));
}

void getFrame(vector<float>& motionFrame)
{
    motionFrame = vector<float>(articulation->GetNDof() / 3 * 4 + 7);
    motionFrame[0] = getsin(pos);
    motionFrame[1] = articulation->linkMap["s0"]->link->getGlobalPose().p.y;
    motionFrame[2] = pos;

    PxQuat prevQuat(PxIdentity);
    for (int i = 3; i < (int)motionFrame.size(); i += 4) {
        PxQuat rotG(atan(getcos(-(i - 3) / 4 * segdist + pos)), PxVec3(0, 1, 0));
        PxQuat rotL = prevQuat.getConjugate() * rotG;
        motionFrame[i] = rotL.w;
        motionFrame[i + 1] = rotL.x;
        motionFrame[i + 2] = rotL.y;
        motionFrame[i + 3] = rotL.z;
        prevQuat = rotG;
    }
}

static float cumulative_t = 0;
void control(PxReal dt) 
{
    if (cumulative_t > 1.0f / frequencyHz) {
        pos += PxPi * 2 / frequencyHz;
        cumulative_t -= 1.0f / frequencyHz;
    }
    cumulative_t += dt;

    vector<float> motionFrame;
    getFrame(motionFrame);

    auto starttime = high_resolution_clock::now();
    switch (controlMethod)
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
        articulation->SetJointVelocitiesPack4(vector<float>(motionFrame.size(), 0));
        break;
    default:
        break;
    }

    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    spdTime += duration;
}

int main(int argc, char** argv)
{
    cxxopts::Options opts("Example", "Pose tracking");
    opts.add_options()
        ("p,performance", "Run performance test")
        ("gen", "generate motion")
        ("long", "Use long snake")
        ("c,control", "Controller", cxxopts::value<int>()->default_value("0"))
        ("t,dt", "Time step", cxxopts::value<float>()->default_value("0.033"))
        ("hz", "control frequency", cxxopts::value<float>()->default_value("30"))
        ("h,height", "height", cxxopts::value<float>()->default_value("0.251"));
    
    auto result = opts.parse(argc, argv);
    controlMethod = result["control"].as<int>();
    longSnake = result["long"].as<bool>();
    height = result["height"].as<float>();
    frequencyHz = result["hz"].as<float>();

    float dt = result["dt"].as<float>();
    initPhysics(dt);

    vector<float> motionFrame;
    getFrame(motionFrame);
    articulation->SetJointPositionsQuaternion(motionFrame);

    if (result["gen"].as<bool>()) {
        vector<vector<float>> frames(frequencyHz);
        for (int i = 0; i< frequencyHz; i++) {
            vector<float> tmp;
            getFrame(tmp);
            frames[i].push_back(2.f / frequencyHz);
            for (float x : tmp) frames[i].push_back(x);
            pos += PxPi * 2 / frequencyHz;
        }
        
        json j;
        j["Frames"] = frames;
        
        string name = "slither";
        if (longSnake) {
            name += "_long";
        }

        string path = "../resources/motions/" + name + ".txt";
        ofstream outputj(path);

        outputj << setw(4) << j << endl;
        outputj.close();

        return 0;
    }

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
        UR_Init(scene->timeStep, "172.27.9.125", 8080, "172.27.59.248", 8081);
        UR_AddArticulation(articulation);
        UR_InitPrimitives();
        for (;;) {
            control(scene->timeStep);
            scene->Step();
            UR_Tick();
        }
        cleanupPhysics();
    }
}
