%{
    #include "Actor.h"
%}

class Actor
{
// API BEGIN
// API END
};

class RigidActor :public Actor
{
// API BEGIN
public:
    void SetupCollisionFiltering(int collisionGroup, int collisionMask);
// API END
};

class RigidActorStatic :public RigidActor
{
// API BEGIN
// API END
};
