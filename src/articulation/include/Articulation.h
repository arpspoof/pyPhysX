#pragma once

#include "PxPhysicsAPI.h"
#include "ArticulationElements.h"
#include "IDisposable.h"

#include <unordered_map>
#include <string>

/**
 * @brief Featherstone generalized coordinate articulation
 * 
 */
class Articulation :public IDisposable
{
// API BEGIN
public:
    /**
     * @brief Get the number of total degrees of freedom in this articulation
     * 
     * @return The number of degrees of freedom
     */
    int GetNDof() const;
    /**
     * @brief Get the number of joints having degree of freedom at least 1
     * 
     * @return The number of joints having degree of freedom at least 1
     */
    int GetNActiveJoints() const;
    /**
     * @brief Set whether or not to fix the articulation base
     * 
     * @param shouldFixBase (true / false)
     */
    void SetFixBaseFlag(bool shouldFixBase);
    /**
     * @brief Get the all Joint degrees of freedom. Result is packed in an
     *  array of Articulation::GetNDof elements and ordered by internal joint id.
     * 
     * @return array of joint defrees of freedom.
     */
    const std::vector<int>& GetJointDofsInIdOrder() const;
    /**
     * @brief Get the Link name specified in the Urdf description.
     * 
     * @param name Link name (string).
     * @return Pointer to Link object.
     */
    const Link* GetLinkByName(std::string name) const;
    /**
     * @brief Get the Joint name specified in the Urdf description.
     * 
     * @param name Joint name (string).
     * @return Pointer to Joint object.
     */
    const Joint* GetJointByName(std::string name) const;
public:
    /**
     * @brief Joint proportional gains.
     * @note All joint gains are packed into one single array here.
     * @note Joint::cacheIndex will tell the start index of entry
     *  corresponding to this joint.
     * 
     */
    std::vector<float> kps;
    /**
     * @brief Joint derivative gains
     * @note All joint gains are packed into one single array here.
     * @note Joint::cacheIndex will tell the start index of entry
     *  corresponding to this joint.
     * 
     */
    std::vector<float> kds;
    /**
     * @brief Joint force limits
     * @note All joint force limits are packed into one single array here.
     * @note Joint::cacheIndex will tell the start index of entry
     *  corresponding to this joint.
     * @note Here force means generalized force or the cartesian force in
     *  joint local frame (Featherstone's articulation).
     * 
     */
    std::vector<float> forceLimits;
    /**
     * @brief Set joint proportional gains through a single array
     * 
     * @param kps A single array which packs all joint proportional gains.
     * @note Joint::cacheIndex will tell the start index of entry
     *  corresponding to this joint.
     */
    void SetKPs(const std::vector<float>& kps);
    /**
     * @brief Set joint derivative gains through a single array
     * 
     * @param kds A single array which packs all joint derivative gains.
     * @note Joint::cacheIndex will tell the start index of entry
     *  corresponding to this joint.
     */
    void SetKDs(const std::vector<float>& kds);
    /**
     * @brief Set joint force limits through a single array
     * 
     * @param forceLimits A single array which packs all joint force limits.
     * @note Joint::cacheIndex will tell the start index of entry
     *  corresponding to this joint.
     */
    void SetForceLimits(const std::vector<float>& forceLimits);
    /**
     * @brief Compute and apply stable PD control to all joints in articulation.
     * @note Joint force is cleared before applying control forces.
     * 
     * @param targetPositions A packed array specifying target positions for all
     *  joints. Joint order is the same as the order used for any other joint
     *  information (like kp, kd, ...). Note that this array does not have to have
     *  Articulation::GetNDof entries. Target position for a 1-dof joint is 
     *  represented by a single angle (in radians) while target position for a
     *  3-dof joint is represented by a quaternion in **WXYZ** order.
     * 
     * @param timeStep Simulation time step is required. Get this via Scene::timeStep.
     */
    void AddSPDForces(const std::vector<float>& targetPositions, float timeStep); // WXYZ or angle
// API END
public:
	std::unordered_map<std::string, Link*> linkMap;
	std::unordered_map<std::string, Joint*> jointMap;
public:
    void SetKPs(const float kps[]);
    void SetKDs(const float kds[]);
    void SetForceLimits(const float forceLimits[]);
    void AddSPDForces(const float targetPositions[], float timeStep); // WXYZ or angle
private:
    std::vector<Joint*> jointList;
    std::vector<int> jointDofs;
    std::vector<std::string> jointNames;
private:
    physx::PxArticulationCache* mainCache;
    physx::PxArticulationCache* massMatrixCache;
    physx::PxArticulationCache* coriolisCache;
    physx::PxArticulationCache* gravityCache;
    physx::PxArticulationCache* externalForceCache;
public:
    physx::PxArticulationReducedCoordinate* pxArticulation;
public:
    Link* AddLink(std::string name, Link *parent, physx::PxTransform transform, LinkBody *body);
	Joint* AddSpericalJoint(std::string name, Link *link,
		physx::PxTransform parentPose, physx::PxTransform childPose);
	Joint* AddRevoluteJoint(std::string name, Link *link, physx::PxArticulationAxis::Enum axis,
		physx::PxTransform parentPose, physx::PxTransform childPose);
	Joint* AddFixedJoint(std::string name, Link *link, 
		physx::PxTransform parentPose, physx::PxTransform childPose);
public:
    void InitControl();
    void Dispose() override;
private:
    void AssignIndices();
};
