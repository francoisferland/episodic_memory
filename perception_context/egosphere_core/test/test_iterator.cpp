#include <gtest/gtest.h>
#include "egosphere/iterator.hpp"

#include <list>

using namespace egosphere;

// TODO check comparision between const and non const
// TODO check behavior when itered element is modified while using the iterator
//      ie. it = list, list << e, it?

//typedef InfoItrStdAdapter<std::list<InfoPtr>::iterator> InfoListItr;




TEST(TestIteratorStd, Iteration)
{
	std::list<int> list;
	list.push_back(1);
	list.push_back(2);
	list.push_back(3);

	Iterator<int> it(list.begin(), list.end());

	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 1);
	it++;
	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 2);
	it++;
	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 3);
}

TEST(TestIteratorStd, TestEnd)
{
	std::list<int> list;
	list.push_back(1);
	list.push_back(2);

	Iterator<int> it(list.begin(), list.end());
	EXPECT_TRUE(it);
	for(int i = 0; i < 3; i++) { it++; }
	EXPECT_FALSE(it);
}

TEST(TestIteratorStd, TestEndOverflow)
{
	std::list<int> list;
	list.push_back(1);
	list.push_back(2);

	Iterator<int> it(list.begin(), list.end());
	EXPECT_TRUE(it);
	for(int i = 0; i < 5; i++) { it++; }
	EXPECT_FALSE(it);
}

TEST(TestIteratorStd, Copy)
{
	std::list<int> list;
	list.push_back(1);
	list.push_back(2);

	Iterator<int> i1(list.begin(), list.end());
	Iterator<int> i2 = i1;

	EXPECT_TRUE(i1 && i2);
	EXPECT_EQ(*i1, *i2);
	i1++;
	EXPECT_NE(*i1, *i2);
	i2++;
	EXPECT_EQ(*i1, *i2);
}

TEST(TestIteratorStd, Count)
{
	std::list<int> list;

	list.push_back(1);
	list.push_back(2);

	Iterator<int> i1(list.begin(), list.end());
	Iterator<int> i2(list.end(), list.end());

	EXPECT_EQ(2, i1.count());
	EXPECT_EQ(2, i1.count());
	EXPECT_EQ(0, i2.count());
}

TEST(TestIteratorTable, Iteration)
{
	// int list[] = {...} fails beacause
	// its type is int[3] and can't be used as initializer
	int * list = (int[]){1, 2, 3};

	Iterator<int> it(list, list+3);

	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 1);
	it++;
	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 2);
	it++;
	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 3);
	it++;
	EXPECT_FALSE(it);
}

TEST(TestPointerIteratorStd, Simple)
{
	int un=1, deux=2, trois=3;
	std::list<int*> list;
	list.push_back(&un);
	list.push_back(&deux);
	list.push_back(&trois);

	PointerIterator<int> it(list.begin(), list.end());

	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 1);
	it++;
	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 2);
	it++;
	EXPECT_TRUE(it);
	EXPECT_EQ(*it, 3);
}

TEST(TestPointerIteratorStd, Copy)
{
	int un=1, deux=2;
	std::list<int*> list;
	list.push_back(&un);
	list.push_back(&deux);

	PointerIterator<int> i1(list.begin(), list.end());
	PointerIterator<int> i2 = i1;

	EXPECT_TRUE(i1 && i2);
	EXPECT_EQ(*i1, *i2);
	i1++;
	EXPECT_NE(*i1, *i2);
}

int main(int argc, char *argv[])
{
	testing::InitGoogleTest(&argc, argv);
	//ros::Time::init();
	return RUN_ALL_TESTS();
}
