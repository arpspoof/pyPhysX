#pragma once

#include "PxPhysicsAPI.h"
#include "IDisposable.h"
#include "Articulation.h"
#include "PrimitiveObjects.h"
#include "ArticulationTree.h"
#include "UrdfLoader.h"
#include "MathInterface.h"
#include "Actor.h"

#include <vector>
#include <unordered_set>

// API BEGIN
/**
 * @brief Description of properties of the scene in creation stage.
 * 
 */
struct SceneDescription
{
    /**
     * @brief Gravity. Default is -9.81
     * 
     */
    float gravity;
    /**
     * @brief Number of extra worker threads used for paralleled simulation.
     *  Default is 0.
     * @note If you're just simulating one articulation (not along with many
     *  other objects), set this to 0 to achieve best performance. Try tuning
     *  this parameter when the number of objects grows in this scene.
     * 
     */
    int nWorkerThreads;
    /**
     * @brief Choose whether to enable GPU dynamics. Default is false.
     * @note Set this to false until PhysX fixes all issues.
     * 
     */
    bool enableGPUDynamics;
    /**
     * @brief Choose whether to enable GPU broad phase. Default is false.
     * @note Set this to false until PhysX fixes all issues.
     * 
     */
    bool enableGPUBroadPhase;
    /**
     * @brief Construct a new SceneDescription object using all default values.
     * 
     */
    SceneDescription();
};
// API END

class Foundation;

/**
 * @brief Scene
 * 
 */
class Scene : public physx::PxSimulationEventCallback, public IDisposable
{
// API BEGIN
public:
    /**
     * @brief Simulation time step.
     * 
     */
    float timeStep;
public:
    /**
     * @brief Create a Material object
     * 
     * @param staticFriction Static friction coefficient
     * @param dynamicFriction Dynamic friction coefficient
     * @param restitution Restitution coefficient
     * @return Pointer to a Material object
     */
    Material* CreateMaterial(float staticFriction, float dynamicFriction, float restitution);
    /**
     * @brief Create a static Plane and add it into the scene.
     * 
     * @param material A pointer to a Material object specifying surface properties.
     * @param planeNormal A 3d vector specifying plane normal in global frame.
     * @param distance Distance from the plane to the world origin.
     * @return Pointer to the created Plane object.
     */
    Plane* CreatePlane(Material* material, vec3 planeNormal, float distance);
    /**
     * @brief Create an Articulation and add it into the scene.
     * 
     * @param urdfLoader Pointer to a UrdfLoader. Make sure the loader has already
     *  loaded some files via UrdfLoader::LoadDescriptionFromFile.
     * @param material Pointer to a Material object. This material will be applied
     *  to all link surfaces of this articulation.
     * @param basePosition A 3d vector specifying the position to place the articulation
     *  in world space coordinates.
     * @return Articulation* Pointer to the created Articulation object.
     */
    Articulation* CreateArticulation(UrdfLoader* urdfLoader, Material* material, vec3 basePosition);
    /**
     * @brief Create an Articulation and add it into the scene.
     * 
     * @param urdfFilePath Path to a Urdf file.
     * @param material Pointer to a Material object. This material will be applied
     *  to all link surfaces of this articulation.
     * @param basePosition A 3d vector specifying the position to place the articulation
     *  in world space coordinates.
     * @return Articulation* Pointer to the created Articulation object.
     */
    Articulation* CreateArticulation(std::string urdfFilePath, Material* material, vec3 basePosition);

    Scene();

    /**
     * @brief Step forward the simulation.
     * 
     */
    void Step();
    /**
     * @brief Dispose the scene and everything created inside this scene.
     * @note Usually Foundation will take care of cleaning up unused scenes.
     *  Do not call this unless you know what you're doing. Also, do not
     *  access anything created by this scene after disposing it.
     * 
     */
    void Dispose() override;

    /**
     * @brief Get all pairs of collision groups involving in contacts
     *  generated by a single scene step.
     * 
     * @return Vector of pairs of integers each pair specifying the two
     *  collision groups involving in a contact.
     * @note If contact information is not retrieved after one scene
     *  stepping, it will be lost after the next scene step.
     */
    const std::vector<std::pair<int, int>>& GetAllContactPairs() const;
// API END
public:
    Scene(Foundation* foundation, SceneDescription description, float timeStep);
    physx::PxScene* GetPxScene() const;
private:
    std::unordered_set<Material*> materials;
    std::unordered_set<physx::PxActor*> actors;
    std::unordered_set<Articulation*> articulations;
	void BuildArticulation(Articulation &ar, ArticulationDescriptionNode* startNode,
		Link* parentLink, physx::PxVec3 parentJointPos, physx::PxVec3 parentLinkPos) const;
    Articulation* CreateArticulation(const ArticulationTree* tree, vec3 basePosition);
private:
    const Foundation* foundation;
    physx::PxScene* pxScene;
    physx::PxDefaultCpuDispatcher* pxCpuDispatcher;
private:
    std::vector<std::pair<int, int>> contacts;
    void onContact(const physx::PxContactPairHeader &pairHeader, 
		const physx::PxContactPair *pairs, physx::PxU32 nbPairs) override;
	void onConstraintBreak(physx::PxConstraintInfo * /*constraints*/, 
        physx::PxU32 /*count*/) override {}
	void onWake(physx::PxActor ** /*actors*/, physx::PxU32 /*count*/) override {}
	void onSleep(physx::PxActor ** /*actors*/, physx::PxU32 /*count*/) override {}
	void onTrigger(physx::PxTriggerPair * /*pairs*/, physx::PxU32 /*count*/) override {}
	void onAdvance(const physx::PxRigidBody *const * /*bodyBuffer*/, 
		const physx::PxTransform * /*poseBuffer*/, const physx::PxU32 /*count*/) override {}
public:
    static physx::PxFilterFlags CollisionShader(
        physx::PxFilterObjectAttributes attributes0, physx::PxFilterData filterData0,
        physx::PxFilterObjectAttributes attributes1, physx::PxFilterData filterData1,
        physx::PxPairFlags& pairFlags, const void* /*constantBlock*/, physx::PxU32 /*constantBlockSize*/);
};
