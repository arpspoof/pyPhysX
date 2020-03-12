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
        FrameState frame;
        frame.groups.push_back(GroupState("articulation"));
        GroupState& group = frame.groups[0];
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
                    group.objectStates.push_back(obj);
                }
            }
        }
        return frame;
    }  
};

class CommandHandler : public AbstractCommandHandler
{ 
    virtual void HandleCommand(const Command& cmd) const override
    {
        printf("cls recv cmd: %s\n", cmd.name.c_str());
    }
};

static DataProvider dataProvider;
static CommandHandler commandHandler;

static float timeStep;

void UR_Init(float timeStep)
{
    ::timeStep = timeStep;
    InitRenderController("172.27.9.125", 8080, "172.27.59.248", 8081, 
        (int)round(1.0 / timeStep), &dataProvider, &commandHandler);
}

void UR_Tick()
{
    Tick(timeStep);
}

void UR_Stop()
{
    DisposeRenderController();
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
            if (body->hasGeometry) {
                BodyGeometryData data;
                body->FillBodyGeometryData(data);
                CreatePrimitive(data.type, "articulation", name, data.param0, data.param1, data.param2);
                printf("track name %s : %s, %f, %f, %f\n", name.c_str(), 
                    data.type.c_str(), data.param0, data.param1, data.param2);
            }
        }
    }
}
