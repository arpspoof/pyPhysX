#pragma once

#include "Articulation.h"
#include "Foundation.h"
#include "PrimitiveObjects.h"
#include "Scene.h"

#include <gtest/gtest.h>

class ArticulationTestResources
{
public:
	Foundation* foundation;
	Material* material;
	Articulation* articulation;
	Scene* scene;

	void Init();
	void Dispose();
};
