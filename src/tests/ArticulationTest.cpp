#include "ArticulationTestResources.h"

#include <unordered_map>
#include <unordered_set>
#include <gtest/gtest.h>

using namespace std;
using namespace testing;

class ArticulationTestFixture :public Test
{
public:
    static ArticulationTestResources res;
    static Articulation* articulation;
    static void SetUpTestCase() 
    {
        res.Init();
        articulation = res.articulation; 
    }
    static void TearDownTestCase()
    {
        res.Dispose();
    }
};

ArticulationTestResources ArticulationTestFixture::res;
Articulation* ArticulationTestFixture::articulation;

template <typename T> void set_equal_to_vec(unordered_set<T> a, vector<T> b)
{
    printf("-- set_equal --: two sets should have same size ...\n");
    ASSERT_EQ(a.size(), b.size());
    printf("-- set_equal --: checking set elements one by one\n");
    for (auto &x : b) {
        ASSERT_TRUE(a.find(x) != a.end());
    }
}

TEST_F(ArticulationTestFixture, test_articulation_topology)
{
    auto &linkMap = articulation->linkMap;
    
    printf("base link should have no parent ...\n");
    ASSERT_EQ(linkMap["base"]->parentLink, nullptr);
    
    printf("base link should have 1 child [root] ...\n");
    set_equal_to_vec({ linkMap["root"] }, linkMap["base"]->childLinks);
    
    printf("root link should have parent [base] ...\n");
    ASSERT_EQ(linkMap["root"]->parentLink, linkMap["base"]);
    
    printf("root link should have 3 children [chest, right_hip, left_hip] ...\n");
    set_equal_to_vec({ linkMap["chest"], linkMap["right_hip"], linkMap["left_hip"] }, linkMap["root"]->childLinks);
    
    printf("chest link should have parent [root] ...\n");
    ASSERT_EQ(linkMap["chest"]->parentLink, linkMap["root"]);
    
    printf("chest link should have 3 children [neck, right_shoulder, left_shoulder] ...\n");
    set_equal_to_vec({ linkMap["neck"], linkMap["right_shoulder"], linkMap["left_shoulder"] }, linkMap["chest"]->childLinks);
    
    printf("neck link should have parent [chest] ...\n");
    ASSERT_EQ(linkMap["neck"]->parentLink, linkMap["chest"]);
    
    printf("neck link should have no child ...\n");
    set_equal_to_vec({ }, linkMap["neck"]->childLinks);
    
    printf("right_shoulder link should have parent [chest] ...\n");
    ASSERT_EQ(linkMap["right_shoulder"]->parentLink, linkMap["chest"]);
    
    printf("right_shoulder link should have 1 child [right_elbow] ...\n");
    set_equal_to_vec({ linkMap["right_elbow"] }, linkMap["right_shoulder"]->childLinks);
    
    printf("left_shoulder link should have parent [chest] ...\n");
    ASSERT_EQ(linkMap["left_shoulder"]->parentLink, linkMap["chest"]);
    
    printf("left_shoulder link should have 1 child [left_elbow] ...\n");
    set_equal_to_vec({ linkMap["left_elbow"] }, linkMap["left_shoulder"]->childLinks);
    
    printf("right_elbow link should have parent [right_shoulder] ...\n");
    ASSERT_EQ(linkMap["right_elbow"]->parentLink, linkMap["right_shoulder"]);
    
    printf("right_elbow link should have 1 child [right_wrist] ...\n");
    set_equal_to_vec({ linkMap["right_wrist"] }, linkMap["right_elbow"]->childLinks);
    
    printf("left_elbow link should have parent [left_shoulder] ...\n");
    ASSERT_EQ(linkMap["left_elbow"]->parentLink, linkMap["left_shoulder"]);
    
    printf("left_elbow link should have 1 child [left_wrist] ...\n");
    set_equal_to_vec({ linkMap["left_wrist"] }, linkMap["left_elbow"]->childLinks);
    
    printf("right_wrist link should have parent [right_elbow] ...\n");
    ASSERT_EQ(linkMap["right_wrist"]->parentLink, linkMap["right_elbow"]);
    
    printf("right_wrist link should have no child ...\n");
    set_equal_to_vec({ }, linkMap["right_wrist"]->childLinks);
    
    printf("left_wrist link should have parent [left_elbow] ...\n");
    ASSERT_EQ(linkMap["left_wrist"]->parentLink, linkMap["left_elbow"]);
    
    printf("left_wrist link should have no child ...\n");
    set_equal_to_vec({ }, linkMap["left_wrist"]->childLinks);
    
    printf("right_hip link should have parent [root] ...\n");
    ASSERT_EQ(linkMap["right_hip"]->parentLink, linkMap["root"]);
    
    printf("right_hip link should have 1 child [right_knee] ...\n");
    set_equal_to_vec({ linkMap["right_knee"] }, linkMap["right_hip"]->childLinks);
    
    printf("left_hip link should have parent [root] ...\n");
    ASSERT_EQ(linkMap["left_hip"]->parentLink, linkMap["root"]);
    
    printf("left_hip link should have 1 child [left_knee] ...\n");
    set_equal_to_vec({ linkMap["left_knee"] }, linkMap["left_hip"]->childLinks);
    
    printf("right_knee link should have parent [right_hip] ...\n");
    ASSERT_EQ(linkMap["right_knee"]->parentLink, linkMap["right_hip"]);
    
    printf("right_knee link should have 1 child [right_ankle] ...\n");
    set_equal_to_vec({ linkMap["right_ankle"] }, linkMap["right_knee"]->childLinks);
    
    printf("left_knee link should have parent [left_hip] ...\n");
    ASSERT_EQ(linkMap["left_knee"]->parentLink, linkMap["left_hip"]);
    
    printf("left_knee link should have 1 child [left_ankle] ...\n");
    set_equal_to_vec({ linkMap["left_ankle"] }, linkMap["left_knee"]->childLinks);
    
    printf("right_ankle link should have parent [right_knee] ...\n");
    ASSERT_EQ(linkMap["right_ankle"]->parentLink, linkMap["right_knee"]);
    
    printf("right_ankle link should have no child ...\n");
    set_equal_to_vec({ }, linkMap["right_ankle"]->childLinks);
    
    printf("left_ankle link should have parent [left_knee] ...\n");
    ASSERT_EQ(linkMap["left_ankle"]->parentLink, linkMap["left_knee"]);
    
    printf("left_ankle link should have no child ...\n");
    set_equal_to_vec({ }, linkMap["left_ankle"]->childLinks);
}

TEST_F(ArticulationTestFixture, test_articulation_joint_connection)
{
    auto &linkMap = articulation->linkMap;
    auto &jointMap = articulation->jointMap;

    printf("base link should have no inbound joint ...\n");
    ASSERT_EQ(linkMap["base"]->inboundJoint, nullptr);

    vector<string> linkNames = {
        "root", "chest", "neck", "right_shoulder", "left_shoulder", 
        "right_elbow", "left_elbow", "right_wrist", "left_wrist",
        "right_hip", "left_hip", "right_knee", "left_knee", "right_ankle", "left_ankle"
    };

    for (string &linkName : linkNames) {
        printf("%s link should have inbound joint %s ...\n", linkName.c_str(), linkName.c_str());
        ASSERT_EQ(linkMap[linkName]->inboundJoint, jointMap[linkName]);
        printf("%s joint should have child link %s ...\n", linkName.c_str(), linkName.c_str());
        ASSERT_EQ(jointMap[linkName]->childLink, linkMap[linkName]);
    }

    vector<string> parentLinkNames = {
        "base", "root", "chest", "chest", "chest", 
        "right_shoulder", "left_shoulder", "right_elbow", "left_elbow",
        "root", "root", "right_hip", "left_hip", "right_knee", "left_knee"
    };

    for (size_t i = 0; i < linkNames.size(); i++) {
        printf("%s joint should have parent link %s ...\n", linkNames[i].c_str(), parentLinkNames[i].c_str());
        ASSERT_EQ(jointMap[linkNames[i]]->parentLink, linkMap[parentLinkNames[i]]);
    }

    printf("root joint should be base ...\n");
    ASSERT_EQ(linkMap["base"], articulation->GetRootLink());
}

TEST_F(ArticulationTestFixture, test_articulation_joint_params)
{
    auto &jointMap = articulation->jointMap;

    unordered_map<string, tuple<int, int, int>> id_dof_CacheIndexList = { 
        { "root", make_tuple(0, 0, -1) },
        { "chest", make_tuple(1, 3, 0) },
        { "neck", make_tuple(2, 3, 12) },
        { "right_hip", make_tuple(3, 3, 3) },
        { "right_knee", make_tuple(4, 1, 18) },
        { "right_ankle", make_tuple(5, 3, 22) },
        { "right_shoulder", make_tuple(6, 3, 15) },
        { "right_elbow", make_tuple(7, 1, 21) },
        { "right_wrist", make_tuple(8, 0, -1) },
        { "left_hip", make_tuple(9, 3, 6) },
        { "left_knee", make_tuple(10, 1, 19) },
        { "left_ankle", make_tuple(11, 3, 25) },
        { "left_shoulder", make_tuple(12, 3, 9) },
        { "left_elbow", make_tuple(13, 1, 20) },
        { "left_wrist", make_tuple(14, 0, -1) }
    };

    for (auto &kvp : id_dof_CacheIndexList) {
        printf("%s joint should have id %d, dof %d, cache index %d ...\n", 
            kvp.first.c_str(), get<0>(kvp.second), get<1>(kvp.second), get<2>(kvp.second));
        ASSERT_EQ(jointMap[kvp.first]->id, get<0>(kvp.second));
        ASSERT_EQ(jointMap[kvp.first]->nDof, get<1>(kvp.second));
        ASSERT_EQ(jointMap[kvp.first]->cacheIndex, get<2>(kvp.second));
    }
}

TEST_F(ArticulationTestFixture, test_articulation_dofs)
{
    printf("total joints should be 15 ...\n");
    ASSERT_EQ(articulation->GetNJoints(), 15);

    printf("total dof should be 28 ...\n");
    ASSERT_EQ(articulation->GetNDof(), 28);

    auto dofs = articulation->GetJointDofsInIdOrder();
    vector<int> expectedDofs = { 0, 3, 3, 3, 1, 3, 3, 1, 0, 3, 1, 3, 3, 1, 0 };

    for (int i = 0; i < (int)expectedDofs.size(); i++) {
        printf("dof of active joint %d should be %d ...\n", i, expectedDofs[i]);
        ASSERT_EQ(dofs[i], expectedDofs[i]);
    }
}

TEST_F(ArticulationTestFixture, test_articulation_params)
{
    int nDof = articulation->GetNDof();

    vector<float> testParams(nDof);
    for (int i = 0; i < nDof; i++)
        testParams[i] = 10 + i;

    articulation->SetKPs(testParams);
    articulation->SetKDs(testParams);
    articulation->SetForceLimits(testParams);

    vector<float> expectedParams = {
        10, 11, 12, 16, 17, 18, 27, 28, 29, 34, 35, 36,
        13, 14, 15, 23, 24, 25, 19, 30, 37, 26, 20, 21, 22, 31, 32, 33
    };
    
    printf("joint params should be correctly set ...\n");
    for (int i = 0; i < nDof; i++) {
        ASSERT_FLOAT_EQ(articulation->kps[i], expectedParams[i]);
        ASSERT_FLOAT_EQ(articulation->kds[i], expectedParams[i]);
        ASSERT_FLOAT_EQ(articulation->forceLimits[i], expectedParams[i]);
    }
}

TEST_F(ArticulationTestFixture, test_kinematic_RW)
{
    vector<float> positions =
    { -0.088583, 2.369318, -2.161314, 0.933701, -0.352505, 0.018137, -0.060115, 0.998819, 0.010960, -0.047140, -0.004159, 0.999996, 0.002537, 0.000256, 0.001070, 0.949948, 0.020403, -0.059407, 0.306028, -0.195258, 0.999520, 0.016056, 0.020116, 0.017256, 0.985617, -0.063945, 0.093094, -0.125710, 0.171284, 0.986347, -0.017107, 0.091650, -0.135749, -0.453371, 0.975329, 0.126891, -0.033021, 0.177601, 0.965989, 0.188903, -0.141940, 0.105041, 0.579958 };
    vector<float> velocities =
    { 0.326730, -2.015506, -2.583195, -1.030780, 0.003017, -0.205119, 0.000000, 0.004144, 0.001047, 0.016726, 0.000000, 0.026550, -0.000165, 0.046535, 0.000000, -0.064505, 0.021811, -0.068365, 0.000000, -0.062028, 0.000961, 0.168048, 0.001768, 0.000000, -0.001193, -0.000624, 0.017256, 0.000000, 0.000266, 0.000804, -0.006196, 0.088094, 0.000000, 0.036677, -0.063728, -0.001808, -0.390550, 0.000000, 0.026291, 0.000256, 0.000952, 0.000000, 0.012650 };

    articulation->SetJointPositionsQuaternion(positions);
    articulation->SetJointVelocitiesPack4(velocities);

    vector<float> newPositions = articulation->GetJointPositionsQuaternion();
    vector<float> newVelocities = articulation->GetJointVelocitiesPack4();

    printf("new and old positions should be equal ...\n");
    for (int i = 0; i < (int)positions.size(); i++) {
        EXPECT_NEAR(positions[i], newPositions[i], 3e-3);
    }

    printf("new and old velocities should be equal ...\n");
    for (int i = 0; i < (int)velocities.size(); i++) {
        EXPECT_NEAR(velocities[i], newVelocities[i], 3e-3);
    }
}
