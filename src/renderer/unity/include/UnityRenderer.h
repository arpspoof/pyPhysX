#pragma once

#include "Articulation.h"

void UR_Init(float timeStep);
void UR_Tick();
void UR_Stop();

void UR_AddArticulation(Articulation* ar);

void UR_InitPrimitives();
