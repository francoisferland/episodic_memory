#include "common.hpp"
#include <gtest/gtest.h>

// TODO test fail when adding / mergin different dimensions parameters

TEST(ParameterInt, NoChallenge)
{
	Parameter douze(12);
	Parameter twelve(12);
	Parameter treize(13);

	EXPECT_EQ(douze.value<int>(), twelve.value<int>());
	EXPECT_EQ(12, twelve.value<int>());

	EXPECT_NE(douze.value<int>(), treize.value<int>());
	EXPECT_NE(12, treize.value<int>());
}

TEST(ParameterString, NoChallenge)
{
	Parameter douze(std::string("douze"));
	EXPECT_EQ("douze", douze.value<std::string>());
}

TEST(Parameter, Defined)
{
	Parameter un('1');
	EXPECT_TRUE(un.defined());
}

TEST(Parameter, Undefined)
{
	EXPECT_FALSE(Parameter::undefined().defined());
}

TEST(ParametersMap, Set)
{
	Params params;
	EXPECT_FALSE(params.contains("douze"));
	params.value("douze", 12);
	EXPECT_TRUE(params.contains("douze"));
	EXPECT_EQ(12, params.value<int>("douze"));
}

TEST(ParametersMap, Update)
{
	Params params;
	params.value("chiffre", 1);
	EXPECT_EQ(1, params.value<int>("chiffre"));
	params.value("chiffre", 2);
	EXPECT_EQ(2, params.value<int>("chiffre"));
}

TEST(ParametersMap, Remove)
{
	Params params;
	params.value("1", 1);
	params.value("2", 2);
	EXPECT_TRUE(params.contains("2"));
	params.remove("2");
	EXPECT_FALSE(params.contains("2"));
	EXPECT_TRUE(params.contains("1"));
}

TEST(ParametersMap, MergeNoCollisions)
{
	Params ab = Params()
		.value("a", 1)
		.value("b", 2);
	Params cd = Params()
		.value("c", 3)
		.value("d", 4);
	
	ab.merge(cd);
	
	EXPECT_EQ(1, ab.value<int>("a"));
	EXPECT_EQ(2, ab.value<int>("b"));
	EXPECT_EQ(3, ab.value<int>("c"));
	EXPECT_EQ(4, ab.value<int>("d"));
}

TEST(ParametersMap, MergeOverride)
{
	Params ab = Params()
		.value("a", 1)
		.value("b", 2);
	Params bc = Params()
		.value("b", 3)
		.value("c", 4);
	
	ab.merge(bc);
	
	EXPECT_EQ(1, ab.value<int>("a"));
	EXPECT_EQ(3, ab.value<int>("b"));
	EXPECT_EQ(4, ab.value<int>("c"));
}

TEST(ParametersMap, NotFound)
{
	Params ab = Params()
		.value("a", 1)
		.value("b", 2);
	EXPECT_THROW(ab.value<int>("c"), ParameterNotDefined);
}

int main(int argc, char *argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
