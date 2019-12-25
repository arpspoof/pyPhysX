#pragma once

#include "Articulation.h"

void UR_Init(int frequency = 100);
void UR_Tick(int slots);
void UR_Stop();

void UR_AddArticulation(Articulation* ar);

void UR_InitPrimitives();
