#include "LinkBody.h"
#include <cassert>

using namespace physx;
using namespace std;

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
    :hasGeometry(true), mass(mass), geometry(geometry)
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
    type = "null";
}

PxGeometry& NULLLinkBody::getGeometry() const 
{
    assert(false);
    return *geometry;
}

#ifdef ENABLE_UNITY_KINEMATICS
void NULLLinkBody::FillCommandParams(const string&, Command&) const
{
    assert(false);
}
#endif


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
    type = "box";
}

#ifdef ENABLE_UNITY_KINEMATICS
void BoxLinkBody::FillCommandParams(const string& name, Command& cmd) const
{
    cmd.ps.clear();
    cmd.ps.push_back("box");
    cmd.ps.push_back(name);
    cmd.pf.clear();
    cmd.pf.push_back(lenX);
    cmd.pf.push_back(lenY);
    cmd.pf.push_back(lenZ);
}
#endif


// SphereLinkBody

float SphereLinkBody::getDensity() const 
{
    return mass / (4.0f / 3.0f*physx::PxPi*radius*radius*radius);
}

SphereLinkBody::SphereLinkBody(float mass, float radius, Material *material)
    :LinkBody(mass, new physx::PxSphereGeometry(radius), material), radius(radius) 
{
    assert(material != nullptr);
    type = "sphere";
}

#ifdef ENABLE_UNITY_KINEMATICS
void SphereLinkBody::FillCommandParams(const string& name, Command& cmd) const
{
    cmd.ps.clear();
    cmd.ps.push_back("sphere");
    cmd.ps.push_back(name);
    cmd.pf.clear();
    cmd.pf.push_back(radius);
}
#endif


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
    type = "capsule";
}

#ifdef ENABLE_UNITY_KINEMATICS
void CapsuleLinkBody::FillCommandParams(const string& name, Command& cmd) const
{
    cmd.ps.clear();
    cmd.ps.push_back("capsule");
    cmd.ps.push_back(name);
    cmd.pf.clear();
    cmd.pf.push_back(radius);
    cmd.pf.push_back(length);
}
#endif
