#pragma once

#include "ArticulationTree.h"
#include "PrimitiveObjects.h"
#include "MathInterface.h"
#include "IDisposable.h"
#include "LinkBody.h"
#include "xml/rapidxml.hpp"

#include <string>
#include <unordered_map>

/**
 * @brief Loader for URDF skeleton files
 * @warning Do not attempt to load files other than humanoid.urdf.
 *  Blame PhysX if this brings any inconvenience...
 * 
 */
class UrdfLoader :public IDisposable
{
// API BEGIN
public:
    /**
     * @brief Construct an empty loader. Call LoadDescriptionFromFile
     *  to load a new URDF file.
     * @param scalingFactor Specify this parameter to scale the model. 
     * 
     */
    UrdfLoader(float scalingFactor = 1.0f);
    /**
     * @brief Dispose all resources allocated by the loader. If a Urdf loader
     *  is created in your code logic, you must call this when it is not needed.
     * 
     */
    void Dispose() override;
    /**
     * @brief Load a new skeleton description.
     * @note Old skeleton description will be discarded.
     * @remark Refer to [Urdf Loader Details](wiki/urdfLoader.md)
     *  for more information about Urdf Loader
     * 
     * @param path File path to URDF skeleton file
     */
    void LoadDescriptionFromFile(std::string path);
// API END
public:
    float scalingFactor;
    std::unordered_map<std::string, int> jointIdMap;
    void BuildArticulationTree(ArticulationTree& tree, Material* material);
private:
    char* buffer;
    rapidxml::xml_document<> doc;
private:
    std::unordered_map<std::string, LinkBody*> linkBodyMap;
    std::unordered_map<std::string, vec3> linkOffsetMap;
    std::unordered_map<std::string, std::string> parentLinkMap;
    void ParseLinkBodies(ArticulationTree& tree, Material* material);
    void ParseLinkOffsets();
    void ParseJoints(ArticulationTree& tree);
};
