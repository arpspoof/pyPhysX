#include "globals.h"
#include "config.h"

using namespace physx;

extern PxReal twistTarget, swing1Target, swing2Target;
extern int xFrame;

void incre(PxReal& x, PxReal v)
{
	x += v;
	while (x > PxPi) x -= 2 * PxPi;
	while (x < -PxPi) x += 2 * PxPi;
}

void keyHandler(unsigned char key, const PxTransform& /*camera*/)
{
	switch (key) {
	case '1':
		writeConfigFile();
		break;
	case 'z': xFrame = PxMax(0, xFrame - 1); printf("xframe = %d\n", xFrame); break;
	case 'x': xFrame = PxMin(95, xFrame + 1); printf("xframe = %d\n", xFrame); break;
	}
}