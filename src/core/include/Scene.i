%{
    #include "Scene.h"
%}

%include "std_vector.i"
%include "std_string.i"

namespace std {
   %template() vector<pair<int, int>>;
};

// API BEGIN
struct SceneDescription
{
    float gravity;
    int nWorkerThreads;
    bool enableGPUDynamics;
    bool enableGPUBroadPhase;

    SceneDescription();
};
// API END

class Foundation;

class Scene : public IDisposable
{
// API BEGIN
public:
    float timeStep;
public:
    Material* CreateMaterial(float staticFriction, float dynamicFriction, float restitution);
    Plane* CreatePlane(Material* material, vec3 planeNormal, float distance);
    Articulation* CreateArticulation(UrdfLoader* urdfLoader, Material* material, vec3 basePosition);
    Articulation* CreateArticulation(std::string urdfFilePath, Material* material, vec3 basePosition);

    Scene();
    void Step();
    void Dispose() override;

    const std::vector<std::pair<int, int>>& GetAllContactPairs() const;
// API END
};
