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
}

TEST_F(ArticulationTestFixture, test_articulation_joint_params)
{
	auto &jointMap = articulation->jointMap;

	unordered_map<string, pair<int, int>> dofAndCacheIndexList = { 
		{ "root", make_pair(0, 0) },
		{ "chest", make_pair(3, 0) },
		{ "right_hip", make_pair(3, 3) },
		{ "left_hip", make_pair(3, 6) },
		{ "neck", make_pair(3, 9) },
		{ "right_shoulder", make_pair(3, 12) },
		{ "left_shoulder", make_pair(3, 15) },
		{ "right_knee", make_pair(1, 18) },
		{ "left_knee", make_pair(1, 19) },
		{ "right_elbow", make_pair(1, 20) },
		{ "left_elbow", make_pair(1, 21) },
		{ "right_ankle", make_pair(3, 22) },
		{ "left_ankle", make_pair(3, 25) }
	};

	for (auto &kvp : dofAndCacheIndexList) {
		printf("%s joint should have dof %d and cache index %d ...\n", kvp.first.c_str(), kvp.second.first, kvp.second.second);
		ASSERT_EQ(jointMap[kvp.first]->nDof, kvp.second.first);
		ASSERT_EQ(jointMap[kvp.first]->cacheIndex, kvp.second.second);
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
