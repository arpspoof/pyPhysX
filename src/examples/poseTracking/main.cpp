
#include <ctype.h>
#include <vector>
#include <stdio.h>
#include <chrono>

#include "PxPhysicsAPI.h"
#include "config.h"
#include "ArticulationTree.h"

#include <iostream>
#include <vector>
#include <algorithm>

#include "MathInterface.h"
#include "PrimitiveObjects.h"
#include "Foundation.h"
#include "UrdfLoader.h"
#include "Scene.h"
#include "Actor.h"

using namespace physx;
using namespace std;
using namespace std::chrono;

Foundation* foundation;
Articulation* articulation;
Scene* scene;
Material* material;

void InitControl();

void initPhysics(bool /*interactive*/)
{
    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), 0.001f);

    material = scene->CreateMaterial(1.f, 1.f, 0.f);

    if (getConfigI("S_GROUND")) {
        auto plane = scene->CreatePlane(material, vec3(0, 1, 0), 0);
        plane->SetupCollisionFiltering(1, 2 | 4);
    }
    
    articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 3.75f, 0), 0.25f);

    articulation->linkMap["right_ankle"]->SetupCollisionFiltering(2, 1 | 4);
    articulation->linkMap["left_ankle"]->SetupCollisionFiltering(4, 1 | 2);

    for (auto &kvp : articulation->jointMap) {
        printf("%s: %d\n", kvp.first.c_str(), kvp.second->cacheIndex);
    }

    InitControl();
}
    
void cleanupPhysics(bool /*interactive*/)
{
    foundation->Dispose();
    delete foundation;
}

vector<float> jp {
    4.202634096145629883e-01,
    7.785608172416687012e-01,
    -5.180309414863586426e-01,
    7.128324508666992188e-01,
    6.410086154937744141e-02,
    -5.739614963531494141e-01,
    -3.979059159755706787e-01,
    9.950842857360839844e-01,
    6.326453387737274170e-02,
    -6.274969875812530518e-02,
    -4.321190714836120605e-02,
    9.896937012672424316e-01,
    8.685035631060600281e-03,
    6.209436804056167603e-02,
    -1.287444233894348145e-01,
    8.434213995933532715e-01,
    3.583949804306030273e-02,
    -3.352406620979309082e-01,
    4.182936549186706543e-01,
    -4.034254848957061768e-01,
    9.619362354278564453e-01,
    -5.770757794380187988e-03,
    -2.653519511222839355e-01,
    6.506693363189697266e-02,
    8.909686803817749023e-01,
    -4.286487400531768799e-01,
    -1.094078719615936279e-01,
    1.022979617118835449e-01,
    4.743066728115081787e-01,
    9.276173114776611328e-01,
    -2.750867605209350586e-02,
    2.718484103679656982e-01,
    2.546912729740142822e-01,
    -4.386751651763916016e-01,
    8.900240063667297363e-01,
    -2.056919634342193604e-01,
    2.583438158035278320e-01,
    3.143349289894104004e-01,
    9.874850511550903320e-01,
    1.325912326574325562e-01,
    -6.863811612129211426e-02,
    5.081036686897277832e-02,
    5.108727216720581055e-01
};

vector<float> jposcmp {
    4.202634096145629883e-01, 7.785608172416687012e-01, -5.180309414863586426e-01,
    5.368506023616530509e-01, 9.379920068199532057e-01, -3.885843697659928098e-01,
    6.209561178718167040e-01, 1.086916972979512064e+00, -2.440986070390926688e-01,
    3.464720755995077672e-01, 8.095767445335443346e-01, -4.897704836407399398e-01,
    1.386283482790451882e-01, 4.620781874600353167e-01, -3.725238849845435851e-01,
    -1.976151046972557390e-02, 8.613609856918019725e-02, -4.121926886027740822e-01,
    4.632271938586635107e-01, 1.181911831490832121e+00, -2.198163718808472666e-01,
    1.986730407733662274e-01, 1.132400159039079313e+00, -2.752068893007557637e-01,
    -3.672258451758070308e-02, 1.043413932341042960e+00, -2.141832987561598789e-01,
    4.940547436296182093e-01, 7.475448899497930677e-01, -5.462913993319773454e-01,
    3.503254284915613415e-01, 3.551428300474334110e-01, -6.016405751238952382e-01,
    1.029606963314431411e-01, 6.953226438161308476e-02, -7.604822737476019112e-01,
    7.994750687019215318e-01, 1.049423785127702624e+00, -2.789885248854427902e-01,
    7.606175478683008251e-01, 8.273987984595271294e-01, -4.361618079333071663e-01,
    6.995390967267837157e-01, 5.771353780202762573e-01, -4.624523315396146250e-01
};

PxReal motions[98][36];

#include <fstream>
#include "GlutRenderer.h"

int xFrame = 0;

extern void control(PxReal dt);
class GlutHandler :public glutRenderer::GlutRendererCallback
{
    void keyboardHandler(unsigned char key) override
    {
        switch (key) {
        case 'z': xFrame = PxMax(0, xFrame - 1); printf("xframe = %d\n", xFrame); break;
        case 'x': xFrame = PxMin(95, xFrame + 1); printf("xframe = %d\n", xFrame); break;
        }
    }
    void beforeSimulationHandler() override
    {
        control(scene->timeStep);
    }
} glutHandler;

int main(int argc, char** argv)
{
    if (argc > 1) {
        const char* config_path = argv[1];
        readConfigFile(config_path);
    }
    else {
        printf("no config file specified\n");
    }

    ifstream motioninput("resources/testMotion.txt");
    for (int i = 0; i < 32; i++) {
        for (int j = 0; j < 43; j++) {
            PxReal tmp;motioninput >> tmp;
            if (j < 7) continue;
            motions[i][j - 7] = tmp;
        }
    }
    motioninput.close();

#if 1
    initPhysics(false);
    articulation->SetJointPositionsQuaternion(jp);
    auto jpos = articulation->CalculateJointPositionsInIdOrder(jp);
    printf("joint positions\n");
    for (vec3 v : jpos) {
        printf("%f, %f, %f\n", v.x, v.y, v.z);
    }
    printf("\n");
    printf("abs err\n");
    for (int i = 0; i < 15; i++) {
        printf("%f, ", jpos[i].x - jposcmp[i * 3]);
        printf("%f, ", jpos[i].y - jposcmp[i * 3 + 1]);
        printf("%f, ", jpos[i].z - jposcmp[i * 3 + 2]);
        printf("\n");
    }
    printf("\n");
    printf("rel err\n");
    for (int i = 0; i < 15; i++) {
        printf("%f, ", PxAbs(jpos[i].x - jposcmp[i * 3]) / jposcmp[i * 3]);
        printf("%f, ", PxAbs(jpos[i].y - jposcmp[i * 3 + 1]) / jposcmp[i * 3 + 1]);
        printf("%f, ", PxAbs(jpos[i].z - jposcmp[i * 3 + 2]) / jposcmp[i * 3 + 2]);
        printf("\n");
    }
    printf("\n");
    auto renderer = glutRenderer::GlutRenderer::GetInstance();
    renderer->AttachScene(scene, &glutHandler);
    renderer->StartRenderLoop();
    cleanupPhysics(false);
#else
    static const PxU32 frameCount = 10000;
    initPhysics(false);
    auto starttime = high_resolution_clock::now();
    for(PxU32 i=0; i<frameCount; i++) {
        control(scene->timeStep);
        scene->Step();
    }
    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    printf("%lld\n", duration);
    cleanupPhysics(false);
#endif
    return 0;
}
