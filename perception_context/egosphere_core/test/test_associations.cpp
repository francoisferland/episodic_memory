#include "egosphere/associations.hpp"
#include <gtest/gtest.h>

#include <list>

using namespace egosphere;

TEST(AssociationsBase, Associate)
{
	Associations<int> a;

	EXPECT_FALSE(a.associated(1,2));
	EXPECT_FALSE(a.associated(3,4));

	a.associate(1,2);
	a.associate(3,4);

	EXPECT_TRUE(a.associated(1,2));
	EXPECT_TRUE(a.associated(2,1));
	EXPECT_TRUE(a.associated(4,3));

	EXPECT_FALSE(a.associated(1,3));
}

TEST(AssociationsBase, Dissociate)
{
	Associations<int> a;

	a.associate(1,2);
	EXPECT_TRUE(a.associated(1,2));

	a.dissociate(1,2);
	EXPECT_FALSE(a.associated(1,2));
}

TEST(AssociationsBase, Redondancy)
{
	Associations<int> a;

	a.associate(1,2);
	a.associate(1,2);
	EXPECT_TRUE(a.associated(1,2));

	a.dissociate(1,2);
	EXPECT_FALSE(a.associated(1,2));
}

TEST(Associations, Iteration)
{
	Associations<int> a;

	a.associate(1,2);
	a.associate(1,2);
	a.associate(1,3);
	a.associate(3,2);
	a.associate(4,1);
	a.associate(2,4);

	// Builds a lists of ints associated with 1
	std::list<int> list(a.begin(1), a.end(1));
	EXPECT_EQ(3, list.size());

	// Finding elements associated with 1
	EXPECT_EQ(2, *std::find(a.begin(1), a.end(1), 2));
	EXPECT_EQ(3, *std::find(a.begin(1), a.end(1), 3));
	EXPECT_EQ(4, *std::find(a.begin(1), a.end(1), 4));
}

bool is_even(int i)
{
	return i % 2 == 0;
}

TEST(Associations, Filter)
{
	Associations<int> a;

	a.associate(1,2);
	a.associate(1,2);
	a.associate(1,3);
	a.associate(3,2);
	a.associate(4,1);
	a.associate(2,4);

	std::list<int> list(a.begin(1, &is_even), a.end(1, &is_even));
	EXPECT_EQ(2, list.size());
	EXPECT_EQ(2, *std::find(list.begin(), list.end(), 2));
	EXPECT_EQ(4, *std::find(list.begin(), list.end(), 4));
}

int main(int argc, char *argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
