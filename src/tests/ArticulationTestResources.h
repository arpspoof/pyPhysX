#include "Articulation.h"
#include "Scene.h"

#include <gtest/gtest.h>

class ArticulationTestResources
{
public:
	physx::PxMaterial* material;
	Articulation* articulation;
	Scene* scene;

	void Init();
	void Dispose();
};
