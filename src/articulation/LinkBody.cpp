#include "LinkBody.h"
#include <cassert>

using namespace physx;

// LinkBody

PxGeometry& LinkBody::getGeometry() const
{
    return *geometry;
}

LinkBody::~LinkBody()
{
    delete geometry;
}

LinkBody::LinkBody(float mass, PxGeometry *geometry, Material *material) 
	:mass(mass), hasGeometry(true), geometry(geometry)
{
    this->material = material ? material->pxMaterial : nullptr;
}


// NULLLinkBody

float NULLLinkBody::getDensity() const 
{
    return 1;
}

NULLLinkBody::NULLLinkBody() :LinkBody(1, nullptr, nullptr) 
{
    hasGeometry = false;
}

PxGeometry& NULLLinkBody::getGeometry() const 
{
    assert(false);
    return *geometry;
}


// BoxLinkBody

float BoxLinkBody::getDensity() const 
{
	return mass / (lenX * lenY * lenZ);
}

BoxLinkBody::BoxLinkBody(float mass, float lenX, float lenY, float lenZ, Material *material)
    :LinkBody(mass, new PxBoxGeometry(lenX / 2, lenY / 2, lenZ / 2), material),
    lenX(lenX), lenY(lenY), lenZ(lenZ)
{
    assert(material != nullptr);
}


// SphereLinkBody

float SphereLinkBody::getDensity() const 
{
    return mass / (4.0f / 3.0f*physx::PxPi*radius*radius*radius);
}

SphereLinkBody::SphereLinkBody(float mass, float radius, Material *material)
    :LinkBody(mass, new physx::PxSphereGeometry(radius), material), radius(radius) 
{
    assert(material != nullptr);
}


// CapsuleLinkBody

float CapsuleLinkBody::getDensity() const 
{
    return mass / (
        4.0f / 3.0f*physx::PxPi*radius*radius*radius +
        4 * physx::PxPi*radius*radius*length
    );
}

CapsuleLinkBody::CapsuleLinkBody(float mass, float radius, float length, Material *material)
    :LinkBody(mass, new physx::PxCapsuleGeometry(radius, length / 2), material), 
    radius(radius), length(length) 
{
    assert(material != nullptr);
}
