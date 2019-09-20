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
public: // components
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
    /**
     * @brief Get the root Link of the Articulation
     * 
     * @return Pointer to root Link object
     */
    const Link* GetRootLink() const;
public: // kinematic
    /**
     * @brief Get root position, orientation, and all joints' orientation
     *  in quaternion representation. Root global translational position
     *  is in result[0-2], root global rotational position (quaternion)
     *  is in result[3-6]. All other joints' position info are placed into
     *  the result by joint order (order of a joint can be obtained by
     *  Joint::jointOrder). A one-dof joint takes up one entry in result
     *  array representing the joint angle (radians). A three-dof joint takes
     *  up four entries in result array representing the joint local frame
     *  rotaion in quaternion representaion in WXYZ order.
     * 
     * @return All joint position information packed into one single array.
     */
    std::vector<float> GetJointPositionsQuaternion() const;
    /**
     * @brief Set root position, orientation, and all joints' orientation
     *  in quaternion representation from a packed array. See
     *  Articulation::GetJointPositionsQuaternion for details about the
     *  structure of this array.
     * 
     * @param positions 
     */
    void SetJointPositionsQuaternion(const std::vector<float>& positions) const;
    /**
     * @brief Get root linear velocity, root angular velocity and all joints'
     *  angular velocities. Root linear velocity is in result[0-2], root
     *  angular velocity in global frame is in result[3-5]. Result[6] is
     *  always zero. All other joints' velocity info are placed into the
     *  result by joint order (order of a joint can be obtained by
     *  Joint::jointOrder). A one-dof joint takes up one entry in result
     *  array representing the joint velocity. A three-dof joint takes
     *  up four entries in result array. First three of these four entries
     *  represent the joint angular velocity (cartesian) in joint frame.
     *  The forth entry is always zero.
     * 
     * @return All joint velocity information packed into one single array.
     */
    std::vector<float> GetJointVelocitiesPack4() const;
    /**
     * @brief Set root linear velocity, root angular velocity and all joints'
     *  angular velocities from a packed array. See
     *  Articulation::GetJointVelocitiesPack4 for details about the
     *  structure of this array.
     * 
     * @param velocities 
     */
    void SetJointVelocitiesPack4(const std::vector<float>& velocities) const;
public: // control
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
     * @remark Number of parameters needed for each joint is the same
     *  as the degree of freedom of that joint.
     */
    void SetKPs(const std::vector<float>& kps);
    /**
     * @brief Set joint derivative gains through a single array
     * 
     * @param kds A single array which packs all joint derivative gains.
     * @note Joint::cacheIndex will tell the start index of entry
     *  corresponding to this joint.
     * @remark Number of parameters needed for each joint is the same
     *  as the degree of freedom of that joint.
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
    int nSphericalJoint, nRevoluteJoint;
public:
    void SetKPs(const float kps[]);
    void SetKDs(const float kds[]);
    void SetForceLimits(const float forceLimits[]);
    void AddSPDForces(const float targetPositions[], float timeStep); // WXYZ or angle
public:
    void FetchKinematicData() const;
private:
    Link* rootLink;
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
