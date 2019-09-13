#include "ArticulationTestResources.h"
#include "ArticulationDescriptionNode.h"
#include "ArticulationTree.h"
#include "UrdfLoader.h"
#include "LinkBody.h"

using namespace std;
using namespace physx;
using namespace testing;

void ArticulationTestResources::Init() {
    printf("-- ArticulationTestFixture -- Initializing ...\n");

    foundation = new Foundation();
    scene = foundation->CreateScene(SceneDescription(), 0.001f);
    material = scene->CreateMaterial(1, 1, 0);

    printf("-- ArticulationTestFixture -- Build articulation ...\n");
    articulation = scene->CreateArticulation("resources/humanoid.urdf", material, vec3(0, 3.75f, 0));
}

void ArticulationTestResources::Dispose()
{
    printf("-- ArticulationTestFixture -- Clean up ...\n");
    foundation->Dispose();
    delete foundation;
}
