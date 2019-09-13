#include "ArticulationTestResources.h"
#include "MathInterface.h"
#include "PxPhysicsAPI.h"
#include "Foundation.h"
#include "Actor.h"

#include <chrono>
#include <gtest/gtest.h>

using namespace std;
using namespace physx;
using namespace testing;
using namespace chrono;

float targetPositions[36] = {
    0.9988130000f, 0.0094850000f, -0.0475600000f, -0.0044750000f, // 0-3
    0.9649400000f, 0.0243690000f, -0.0575550000f, 0.2549220000f, //8-11
    0.9927550000f, -0.0209010000f, 0.0888240000f, -0.0781780000f, //22-25
    0.9659420000f, 0.1884590000f, -0.1422460000f, 0.1058540000f, //31-34
    1.0000000000f,0.0000000000f, 0.0000000000f, 0.0000000000f, //4-7
    0.9854980000f, -0.0644070000f, 0.0932430000f, -0.1262970000f, //17-20
    -0.2491160000f, //12
    -0.3915320000f, //26
    0.5813480000, //35
    0.1705710000f, //21
    0.9993660000f, 0.0099520000f, 0.0326540000f, 0.0100980000f, //13-16
    0.9828790000f, 0.1013910000 -0.0551600000f, 0.1436190000f, //27-30
};

class PerformanceIntegrationTestFixture :public Test
{
public:
    static ArticulationTestResources res;
    static void SetUpTestCase()
    {
        printf("-- PerformanceIntegrationTestFixture -- Set up ...\n");

        res.Init();

		res.scene->CreatePlane(res.material, vec3(0, 1, 0), 0);

        vector<float> kps(res.articulation->GetNDof(), 10000.f);
        vector<float> kds(res.articulation->GetNDof(), 400.f);
        res.articulation->SetKPs(kps.data());
        res.articulation->SetKDs(kds.data());
    }
    static void TearDownTestCase()
    {
        printf("-- PerformanceIntegrationTestFixture -- Tear down ...\n");
        res.Dispose();
    }
};

ArticulationTestResources PerformanceIntegrationTestFixture::res;

#define RENDER_LAST_FRAME 0

#if(RENDER_LAST_FRAME)
#include "GlutRenderer.h"
#endif

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
    printf("simulation time is %lld, this should be less than 690000 ...\n", duration);
    ASSERT_LE(duration, 690000);
#if(RENDER_LAST_FRAME)
	auto renderer = glutRenderer::GlutRenderer::GetInstance();
	renderer->AttachScene(res.scene);
	renderer->StartRenderLoop();
#endif
}
