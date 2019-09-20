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

    unordered_map<string, tuple<int, int, int>> dof_order_CacheIndexList = { 
        { "root", make_tuple(0, 0, 0) },
        { "chest", make_tuple(3, 1, 0) },
        { "right_hip", make_tuple(3, 2, 3) },
        { "left_hip", make_tuple(3, 3, 6) },
        { "left_shoulder", make_tuple(3, 4, 9) },
        { "neck", make_tuple(3, 5, 12) },
        { "right_shoulder", make_tuple(3, 6, 15) },
        { "right_knee", make_tuple(1, 7, 18) },
        { "left_knee", make_tuple(1, 8, 19) },
        { "left_elbow", make_tuple(1, 9, 20) },
        { "right_elbow", make_tuple(1, 10, 21) },
        { "right_ankle", make_tuple(3, 11, 22) },
        { "left_ankle", make_tuple(3, 12, 25) }
    };

    for (auto &kvp : dof_order_CacheIndexList) {
        printf("%s joint should have dof %d, order %d and cache index %d ...\n", 
            kvp.first.c_str(), get<0>(kvp.second), get<1>(kvp.second), get<2>(kvp.second));
        ASSERT_EQ(jointMap[kvp.first]->nDof, get<0>(kvp.second));
        ASSERT_EQ(jointMap[kvp.first]->jointOrder, get<1>(kvp.second));
        ASSERT_EQ(jointMap[kvp.first]->cacheIndex, get<2>(kvp.second));
    }
}

TEST_F(ArticulationTestFixture, test_articulation_dofs)
{
    printf("n active joints shoule be 12 ...\n");
    ASSERT_EQ(articulation->GetNActiveJoints(), 12);

    printf("total dof should be 28 ...\n");
    ASSERT_EQ(articulation->GetNDof(), 28);

    auto dofs = articulation->GetJointDofsInIdOrder();
    vector<int> expectedDofs = { 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 3, 3 };

    for (int i = 0; i < (int)expectedDofs.size(); i++) {
        printf("dof of active joint %d should be %d ...\n", i, expectedDofs[i]);
        ASSERT_EQ(dofs[i], expectedDofs[i]);
    }
}

TEST_F(ArticulationTestFixture, test_articulation_params)
{
    int nActiveJoint = articulation->GetNActiveJoints();

    vector<float> testParams(nActiveJoint);
    for (int i = 0; i < nActiveJoint; i++)
        testParams[i] = 100 + i;

    articulation->SetKPs(testParams.data());
    articulation->SetKDs(testParams.data());
    articulation->SetForceLimits(testParams.data());
    
    printf("joint params should be correctly set ...\n");
    for (int i = 0; i < nActiveJoint; i++) {
        ASSERT_FLOAT_EQ(articulation->kps[i], 100.f + i);
        ASSERT_FLOAT_EQ(articulation->kds[i], 100.f + i);
        ASSERT_FLOAT_EQ(articulation->forceLimits[i], 100.f + i);
    }
}

TEST_F(ArticulationTestFixture, test_kinematic_RW)
{
    vector<float> positions =
    { -0.010705226100981236, 2.9520351886749268, -1.0793187618255615, 0.9849750995635986, -0.16240181028842926, -0.0562966912984848, -0.01674160361289978, 0.9991642832756042, 0.01748298667371273, -0.03642556071281433, 0.006173908710479736, 0.9542080760002136, 0.026233119890093803, -0.02625342272222042, 0.2968323826789856, 0.9903703331947327, -0.026543904095888138, 0.1235213577747345, -0.0566059947013855, 0.9678494930267334, 0.18189097940921783, -0.1365630179643631, 0.10739383101463318, 0.9999954104423523, -0.0020434020552784204, 0.0005176311242394149, -0.002176225185394287, 0.9860534071922302, -0.06559967249631882, 0.08769472688436508, -0.12531918287277222, -0.3445289134979248, -0.5505771636962891, 0.5630042552947998, 0.16350771486759186, 0.9819461107254028, 0.1464308202266693, 0.020017214119434357, -0.11806392669677734, 0.913760781288147, 0.1983564794063568, -0.000654958188533783, 0.35453569889068604 };
    vector<float> velocities =
    { 0.3752318322658539, -1.256173849105835, -2.068544864654541, -0.794529139995575, 0.01734177954494953, -0.147354394197464, 0.0, 0.0021516273263841867, 0.012157208286225796, -0.03684470057487488, 0.0, -0.00784284621477127, -0.08185304701328278, -0.0393180251121521, 0.0, 0.012893409468233585, 0.0004450793785508722, 0.007343634031713009, 0.0, 0.007491157855838537, 0.005215386860072613, 0.012697052210569382, 0.0, 0.0011133216321468353, -0.00027246022364124656, 0.02609015628695488, 0.0, -0.00166785204783082, -0.01348275039345026, 0.01385579351335764, 0.0, 0.07884275913238525, 0.06096728891134262, 0.0030121812596917152, 0.0030538737773895264, 0.5491054654121399, 0.06237462908029556, -0.20728544890880585, 0.0, 0.3967995345592499, 0.00897566694766283, -0.3366695046424866, 0.0 };

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
        printf("v%d.......\n", i);
        EXPECT_NEAR(velocities[i], newVelocities[i], 1e-3);
    }

    printf("old ...\n");
    for (int i = 0; i < (int)velocities.size(); i++) {
        printf("%f, ", velocities[i]);
    }
    printf("\n");

    printf("new ...\n");
    for (int i = 0; i < (int)newVelocities.size(); i++) {
        printf("%f, ", newVelocities[i]);
    }
    printf("\n");
}
