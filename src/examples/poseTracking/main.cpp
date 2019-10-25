
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

Foundation* foundation;
Articulation* articulation;
Scene* scene;
Material* material;

float kp, kd, rkp, rkd;
vector<vector<float>> motions;
int xFrame = 0;
int nRounds = 0;

void reset()
{
    vector<float> p { 0, 3.55f, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0};
    vector<float> v(43, 0);
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

    float rootKpL = 0*10000.f, rootKdL = 0*1000.f;
    float rootKpA = rkp, rootKdA = rkd;
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
    
    articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 3.75f, 0));

    InitControl();
}
    
void cleanupPhysics()
{
    foundation->Dispose();
    delete foundation;
}

void control(PxReal dt) {
    static int currentRound = 0;

    articulation->AddSPDForcesABA(motions[xFrame], dt, true);
    xFrame = (xFrame + 1) % motions.size();
    
    if (xFrame == 0) {
        currentRound++;
        if (nRounds > 0 && currentRound == nRounds) {
            reset();
            currentRound = 0;
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
        control(scene->timeStep);
    }
} glutHandler;

int main(int argc, char** argv)
{
    cxxopts::Options opts("Example", "Pose tracking");
    opts.add_options()
        ("p,performance", "Run performance test")
        ("r,round", "Number of rounds", cxxopts::value<int>()->default_value("0"))
        ("m,mocap", "Mocap data path", cxxopts::value<string>()->default_value("testMotion.txt"))
        ("t,dt", "Time step", cxxopts::value<float>()->default_value("0.033"))
        ("kp", "Joint kp", cxxopts::value<float>()->default_value("25000"))
        ("kd", "Joint kd", cxxopts::value<float>()->default_value("6600"))
        ("rkp", "Root kp", cxxopts::value<float>()->default_value("2000"))
        ("rkd", "Root kd", cxxopts::value<float>()->default_value("1800"));
    auto result = opts.parse(argc, argv);

    kp = result["kp"].as<float>();
    kd = result["kd"].as<float>();
    rkp = result["rkp"].as<float>();
    rkd = result["rkd"].as<float>();
    nRounds = result["round"].as<int>();

    string mocap = "testMotion.txt";
    if (result["mocap"].as<string>() != "") mocap = result["mocap"].as<string>();

    ifstream motioninput("resources/" + mocap);

    float tmp; int col = 0;
    while (motioninput >> tmp) {
        if (col == 0) motions.push_back(vector<float>(43));
        motions.back()[col] = tmp;
        col = (col + 1) % 43;
    }
    motioninput.close();
    for (auto & r : motions) r[1] += 3.15f;

    if (result["performance"].as<bool>()) {
        static const PxU32 frameCount = 10000;
        initPhysics(result["dt"].as<float>());
        auto starttime = high_resolution_clock::now();
        for(PxU32 i = 0; i < frameCount; i++) {
            control(scene->timeStep);
            scene->Step();
        }
        auto endtime = high_resolution_clock::now();
        auto duration = duration_cast<microseconds>(endtime - starttime).count();
        printf("%ld\n", duration);
        auto renderer = glutRenderer::GlutRenderer::GetInstance();
        renderer->AttachScene(scene, &glutHandler);
        renderer->StartRenderLoop();
        cleanupPhysics();
    }
    else {
        initPhysics(result["dt"].as<float>());
        auto renderer = glutRenderer::GlutRenderer::GetInstance();
        renderer->AttachScene(scene, &glutHandler);
        renderer->StartRenderLoop();
        cleanupPhysics();
    }

    return 0;
}
