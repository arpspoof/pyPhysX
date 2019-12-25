#pragma once

#include "PxPhysicsAPI.h"
#include "PrimitiveObjects.h"

#ifdef ENABLE_UNITY_KINEMATICS
#include <string>
#include <KinematicsClient.hpp>
#endif

class LinkBody {
public:
    std::string type;
    bool hasGeometry;
    float mass;
    physx::PxGeometry* geometry;
    physx::PxMaterial* material;
    virtual physx::PxGeometry& getGeometry() const;
    virtual float getDensity() const = 0;
    virtual ~LinkBody();

#ifdef ENABLE_UNITY_KINEMATICS
    virtual void FillCommandParams(const std::string& name, Command& cmd) const = 0;
#endif
protected:
    LinkBody(float mass, physx::PxGeometry *geometry, Material *material);
};

class NULLLinkBody : public LinkBody {
public:
    NULLLinkBody();
    float getDensity() const override;
    physx::PxGeometry& getGeometry() const override;

#ifdef ENABLE_UNITY_KINEMATICS
    virtual void FillCommandParams(const std::string& name, Command& cmd) const override;
#endif
};

class BoxLinkBody : public LinkBody {
public:
    float lenX, lenY, lenZ;
    float getDensity() const override;
    BoxLinkBody(float mass, float lenX, float lenY, float lenZ, Material *material);

#ifdef ENABLE_UNITY_KINEMATICS
    virtual void FillCommandParams(const std::string& name, Command& cmd) const override;
#endif
};

class SphereLinkBody : public LinkBody {
public:
    float radius;
    float getDensity() const override;
    SphereLinkBody(float mass, float radius, Material *material);

#ifdef ENABLE_UNITY_KINEMATICS
    virtual void FillCommandParams(const std::string& name, Command& cmd) const override;
#endif
};

class CapsuleLinkBody : public LinkBody {
public:
    float radius;
    float length;
    float getDensity() const override;
    CapsuleLinkBody(float mass, float radius, float length, Material *material);

#ifdef ENABLE_UNITY_KINEMATICS
    virtual void FillCommandParams(const std::string& name, Command& cmd) const override;
#endif
};

