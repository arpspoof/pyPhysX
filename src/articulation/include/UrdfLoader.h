#pragma once

#include "ArticulationTree.h"
#include "PrimitiveObjects.h"
#include "MathInterface.h"
#include "IDisposable.h"
#include "LinkBody.h"
#include "xml/rapidxml.hpp"

#include <string>
#include <unordered_map>

class UrdfLoader :public IDisposable
{
public:
    UrdfLoader();
    void Dispose() override;
    void LoadDescriptionFromFile(std::string path);
    void BuildArticulationTree(ArticulationTree& tree, Material* material);
private:
    char* buffer;
    rapidxml::xml_document<> doc;
private:
    std::unordered_map<std::string, LinkBody*> linkBodyMap;
    std::unordered_map<std::string, vec3> linkOffsetMap;
    std::unordered_map<std::string, std::string> parentLinkMap;
    void ParseLinkBodies(ArticulationTree& tree, Material* material);
    void ParseLinkOffsets(ArticulationTree& tree);
    void ParseJoints(ArticulationTree& tree);
};
