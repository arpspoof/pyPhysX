#include "ArticulationTestResources.h"
#include "MathInterface.h"
#include "PxPhysicsAPI.h"
#include "Foundation.h"
#include "Actor.h"

#include <chrono>
#include <vector>
#include <gtest/gtest.h>

using namespace std;
using namespace physx;
using namespace testing;
using namespace chrono;

vector<float> targetPositions = { 0.998819, 0.010960, -0.047140, -0.004159, 0.999996, 0.002537, 0.000256, 0.001070, 0.949948, 0.020403, -0.059407, 0.306028, -0.195258, 0.999520, 0.016056, 0.020116, 0.017256, 0.985617, -0.063945, 0.093094, -0.125710, 0.171284, 0.986347, -0.017107, 0.091650, -0.135749, -0.453371, 0.975329, 0.126891, -0.033021, 0.177601, 0.965989, 0.188903, -0.141940, 0.105041, 0.579958 };

#include "GlutRenderer.h"

class GlutHandler :public glutRenderer::GlutRendererCallback
{
public:
    Articulation* articulation;
    Scene* scene;
    void keyboardHandler(unsigned char /*key*/) override
    {
    }
    void beforeSimulationHandler() override
    {
        articulation->AddSPDForces(targetPositions, scene->timeStep);
    }
};

class PerformanceIntegrationTestFixture :public Test
{
public:
    static GlutHandler glutHandler;
    static ArticulationTestResources res;
    static void SetUpTestCase()
    {
        printf("-- PerformanceIntegrationTestFixture -- Set up ...\n");

        res.Init();

        res.scene->CreatePlane(res.material, vec3(0, 1, 0), 0);

        vector<float> kps(res.articulation->GetNDof(), 10000.f);
        vector<float> kds(res.articulation->GetNDof(), 400.f);
        res.articulation->SetKPs(kps);
        res.articulation->SetKDs(kds);

        glutHandler.scene = res.scene;
        glutHandler.articulation = res.articulation;
    }
    static void TearDownTestCase()
    {
        printf("-- PerformanceIntegrationTestFixture -- Tear down ...\n");
        res.Dispose();
    }
};

ArticulationTestResources PerformanceIntegrationTestFixture::res;
GlutHandler PerformanceIntegrationTestFixture::glutHandler;

#define RENDER_LAST_FRAME 0

TEST_F(PerformanceIntegrationTestFixture, test_performance_integration)
{
    static const PxU32 frameCount = 10000;
    auto starttime = high_resolution_clock::now();
    for(PxU32 i=0; i<frameCount; i++) {
        res.articulation->AddSPDForces(targetPositions, res.scene->timeStep);
        res.scene->Step();
    }
    auto endtime = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(endtime - starttime).count();
    printf("simulation time is %ld, this should be less than 690000 ...\n", duration);
    ASSERT_LE(duration, 690000);
#if(RENDER_LAST_FRAME)
    auto renderer = glutRenderer::GlutRenderer::GetInstance();
    renderer->AttachScene(res.scene, &glutHandler);
    renderer->StartRenderLoop();
#endif
}
