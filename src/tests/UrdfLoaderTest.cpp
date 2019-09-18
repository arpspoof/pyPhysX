#include "ArticulationTree.h"
#include "UrdfLoader.h"
#include "LinkBody.h"
#include "gtest/gtest.h"

#include <string>
#include <unordered_map>

using namespace std;
using namespace testing;

class UrdfLoaderTestFixture : public Test
{
public:
    static UrdfLoader loader;
    static ArticulationTree tree;

    static void SetUpTestCase() 
    {
        loader.LoadDescriptionFromFile("resources/humanoid.urdf");
        loader.BuildArticulationTree(tree, nullptr);
    }
    static void TearDownTestCase() 
    {
        loader.Dispose();
    }
};

UrdfLoader UrdfLoaderTestFixture::loader;
ArticulationTree UrdfLoaderTestFixture::tree;

TEST_F(UrdfLoaderTestFixture, test_urdf_loader_link_bodies)
{
    auto linkBodies = tree.GetLinkBodies();
    printf("number of link bodies should be 16 ...\n");
    ASSERT_EQ(linkBodies.size(), 16);

    auto nodeMap = tree.GetNodeMap();
    printf("node map size should be 16 ...\n");
    ASSERT_EQ(nodeMap.size(), 16);

    printf("checking link bodies' types ...\n");
    ASSERT_TRUE(dynamic_cast<NULLLinkBody*>(nodeMap["base"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<SphereLinkBody*>(nodeMap["root"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<SphereLinkBody*>(nodeMap["chest"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<SphereLinkBody*>(nodeMap["neck"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<SphereLinkBody*>(nodeMap["right_wrist"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<SphereLinkBody*>(nodeMap["left_wrist"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["right_shoulder"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["left_shoulder"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["right_hip"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["left_hip"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["right_knee"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["left_knee"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["right_elbow"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<CapsuleLinkBody*>(nodeMap["left_elbow"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<BoxLinkBody*>(nodeMap["right_ankle"]->body) != nullptr);
    ASSERT_TRUE(dynamic_cast<BoxLinkBody*>(nodeMap["left_ankle"]->body) != nullptr);
}

TEST_F(UrdfLoaderTestFixture, test_urdf_loader_link_body_params)
{
    auto nodeMap = tree.GetNodeMap();

    auto box = dynamic_cast<BoxLinkBody*>(nodeMap["right_ankle"]->body);
    printf("checking swapping x & y for box link ...\n");
    ASSERT_FLOAT_EQ(box->lenX, 0.22f);
    ASSERT_FLOAT_EQ(box->lenY, 0.708f);
    ASSERT_FLOAT_EQ(box->lenZ, 0.36f);

    auto sphere = dynamic_cast<SphereLinkBody*>(nodeMap["neck"]->body);
    printf("checking radius for sphere link ...\n");
    ASSERT_FLOAT_EQ(sphere->radius, 0.41f);

    auto capsule = dynamic_cast<CapsuleLinkBody*>(nodeMap["right_hip"]->body);
    printf("checking radius and length for capsule link ...\n");
    ASSERT_FLOAT_EQ(capsule->radius, 0.22f);
    ASSERT_FLOAT_EQ(capsule->length, 1.2f);
}

TEST_F(UrdfLoaderTestFixture, test_urdf_loader_topology)
{
    auto nodeMap = tree.GetNodeMap();

    vector<string> linkNames = {
        "root", "chest", "neck", "right_shoulder", "left_shoulder", 
        "right_elbow", "left_elbow", "right_wrist", "left_wrist",
        "right_hip", "left_hip", "right_knee", "left_knee", "right_ankle", "left_ankle"
    };

    vector<string> parentLinkNames = {
        "base", "root", "chest", "chest", "chest", 
        "right_shoulder", "left_shoulder", "right_elbow", "left_elbow",
        "root", "root", "right_hip", "left_hip", "right_knee", "left_knee"
    };

    printf("checking description node parent node one by one ...\n");

    for (int i = 0; i < (int)linkNames.size(); i++) {
        ASSERT_EQ(nodeMap[linkNames[i]]->parent, nodeMap[parentLinkNames[i]]);
    }
}
