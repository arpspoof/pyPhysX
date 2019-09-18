#pragma once

#include "PxPhysicsAPI.h"
#include "PrimitiveObjects.h"

class LinkBody {
public:
    bool hasGeometry;
    float mass;
    physx::PxGeometry* geometry;
    physx::PxMaterial* material;
    virtual physx::PxGeometry& getGeometry() const;
    virtual float getDensity() const = 0;
    virtual ~LinkBody();
protected:
    LinkBody(float mass, physx::PxGeometry *geometry, Material *material);
};

class NULLLinkBody : public LinkBody {
public:
    NULLLinkBody();
    float getDensity() const override;
    physx::PxGeometry& getGeometry() const override;
};

class BoxLinkBody : public LinkBody {
public:
    float lenX, lenY, lenZ;
    float getDensity() const override;
    BoxLinkBody(float mass, float lenX, float lenY, float lenZ, Material *material);
};

class SphereLinkBody : public LinkBody {
public:
    float radius;
    float getDensity() const override;
    SphereLinkBody(float mass, float radius, Material *material);
};

class CapsuleLinkBody : public LinkBody {
public:
    float radius;
    float length;
    float getDensity() const override;
    CapsuleLinkBody(float mass, float radius, float length, Material *material);
};

