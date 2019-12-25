#include "UnityRenderer.h"
#include "PxPhysicsAPI.h"

#include <KinematicsClient.hpp>
#include <string>
#include <vector>

using namespace std;
using namespace physx;

static vector<Articulation*> articulations;

class DataProvider : public AbstractDataProvider
{
    virtual FrameState GetCurrentState() const override
    {
        FrameState frame("session");

        for (auto& ar : articulations) {
            for (auto& kvp : ar->linkMap) {
                const string& name = kvp.first;
                Link* link = kvp.second;
                LinkBody* body = link->body;
                if (body->geometry) {
                    PxVec3 p = link->link->getGlobalPose().p;
                    PxQuat q = link->link->getGlobalPose().q;
                    if (body->type == "capsule") {
                        q = q * PxQuat(-PxPi / 2, PxVec3(0, 0, 1));
                    }
                    ObjectState obj(name, p.x, p.y, p.z, q.w, q.x, q.y, q.z);
                    frame.objectStates.push_back(obj);
                }
            }
        }

        return frame;
    }   
};

static DataProvider dataProvider;

void UR_Init(int frequency)
{
    RPCStartClient("localhost", 8080);
    dataProvider.frequency = frequency;
}

void UR_Tick(int slots)
{
    dataProvider.Tick(slots);
}

void UR_Stop()
{
    RPCStopClient();
}

void UR_AddArticulation(Articulation* ar)
{
    articulations.push_back(ar);
}

void UR_InitPrimitives()
{
    for (auto& ar : articulations) {
        for (auto& kvp : ar->linkMap) {
            const string& name = kvp.first;
            Link* link = kvp.second;
            LinkBody* body = link->body;
            if (body->geometry) {
                Command cmd;
                cmd.name = "_sys_create_primitive";
                body->FillCommandParams(name, cmd);
                SendCommand(cmd);
                printf("track name %s\n", name.c_str());
            }
        }
    }
}
