#include "common.hpp"
#include <gtest/gtest.h>
#include <boost/foreach.hpp>

TEST(Tags, NotExpired)
{
  Info::Ptr i = Info::create(0);
  EXPECT_FALSE(i->expired());
}

TEST(Tags, AddOne)
{
  Info::Ptr i = Info::create(0);
  EXPECT_EQ(0, i->tags().size());
  i->set(Tags("tag1"));
  EXPECT_EQ(1, i->tags().size());
}

TEST(Tags, HasTag)
{
  Info::Ptr i = Info::create(0);
  i->set(Tags("tag1"));
  EXPECT_FALSE(i->is("tag0"));
  EXPECT_TRUE(i->is("tag1"));
}

TEST(Tags, AddOneManyTimes)
{
  Info::Ptr i = Info::create(0);
  EXPECT_EQ(0, i->tags().size());
  i->set(Tags("tag1"));
  i->set(Tags("tag2"));
  EXPECT_EQ(2, i->tags().size());
  EXPECT_TRUE(i->is("tag1"));
  EXPECT_TRUE(i->is("tag2"));
}

TEST(Tags, AddMany)
{
  Info::Ptr i = Info::create(0);
  ASSERT_EQ(0, i->tags().size());
  i->set(Tags("tag1, tagdeux, 3rdtag"));
  ASSERT_EQ(3, i->tags().size());
  EXPECT_TRUE(i->is("tag1"));
  EXPECT_TRUE(i->is("tagdeux"));
  EXPECT_TRUE(i->is("3rdtag"));
}

TEST(Tags, AddManyManyTimes)
{
  Info::Ptr i = Info::create(0);
  ASSERT_EQ(0, i->tags().size());
  i->set(Tags("tag1, tagdeux"));
  i->set(Tags("3rdtag"));
  ASSERT_EQ(3, i->tags().size());
  EXPECT_TRUE(i->is("tag1"));
  EXPECT_TRUE(i->is("tagdeux"));
  EXPECT_TRUE(i->is("3rdtag"));
}

TEST(Params, Set)
{
  Info::Ptr i = Info::create(0);
  EXPECT_FALSE(i->has("un"));
  i->set(Params().value("un", 1));
  EXPECT_TRUE(i->has("un"));
  EXPECT_EQ(1, i->get<int>("un"));
}

TEST(Params, Remove)
{
  Info::Ptr i = Info::create(0);
  i->set(Params()
         .value("un", 1)
         .value("deux", 2));
  i->remove("un");
  EXPECT_FALSE(i->has("un"));
  EXPECT_TRUE(i->has("deux"));
}

// Remember 100 seconds or 1 elements
// (first reached limit, 1 element in this case)

TEST(Info, CacheSize1)
{
  Info::Ptr info = Info::create(100, 1);
  EXPECT_EQ(0, info->params().size());
  for(double t = 0; t <= 10; t++) {
    info->set(Params().value("v", t), t);
    // Only one memorised element
    EXPECT_EQ(1, info->params().size());
    // The more recent one
    EXPECT_EQ(t, info->get<double>("v"));
  }
}

// Remember 100 seconds or 5 elements
// (first reached limit)

TEST(Info, CacheSize)
{
  Info::Ptr info = Info::create(100, 5);
  EXPECT_EQ(0, info->params().size());
  for(double t = 0; t < 10; t++) {
    info->set(Params().value("v", t), t);
    // Size should never exceed 5
    EXPECT_TRUE(info->params().size() <= 5);
    // The last value should be the more recent one
    EXPECT_EQ(t, info->get<double>("v"));
  }
}

// Only remeber the last known parameter value

TEST(Info, CacheDuration0)
{
  Info::Ptr info = Info::create(0);
  EXPECT_EQ(0, info->params().size());
  for(double t = 0; t <= 10; t++) {
    info->set(Params().value("v", t), t);
    // Only one memorised element
    EXPECT_EQ(1, info->params().size());
    // The more recent one
    EXPECT_EQ(t, info->get<double>("v"));
  }
}

TEST(Info, CacheDuration)
{
  Info::Ptr info = Info::create(5);
  EXPECT_EQ(0, info->params().size());
  for(double t = 0; t < 1000; t++) {
    info->set(Params().value("v", t), t);
    EXPECT_EQ(t, info->get<double>("v"));
  }
  ASSERT_TRUE(info->params().size() >= 2);
  EXPECT_TRUE(5 >=
    info->params().begin()->first - (--info->params().end())->first);
}

// If a parameter value is at the end of the cache, but is the last
// defined value of this parameter, keep it even it it is
// the oldest one.

TEST(Info, CacheKeepLastVersion)
{
  Info::Ptr info = Info::create(100, 5);
  info->set(Params().value("not_frequently_updated1", 1), 0);
  info->set(Params().value("frequently_updated",      0), 1);
  info->set(Params().value("not_frequently_updated2", 2), 2);
  info->set(Params().value("frequently_updated",      0), 3);
  info->set(Params().value("frequently_updated",      0), 4);
  info->set(Params().value("frequently_updated",      0), 5);
  info->set(Params().value("frequently_updated",      0), 6);
  info->set(Params().value("frequently_updated",      0), 7);
  // Some parameters have been dumped
  EXPECT_EQ(5, info->params().size());
  // But not oldest if they represent their latest version
  EXPECT_EQ(1, info->get<int>("not_frequently_updated1"));
  EXPECT_EQ(2, info->get<int>("not_frequently_updated2"));
}

// Should not compile (don't know how to automate this test)
/*TEST(Info, NonCopyable)
  {
  Info::Ptr i1 = Info::create();
  Info::Ptr i2 = Info::create();
 *i1 = *i2;
 }*/

class StampedParams : public testing::Test
{
 protected:
  void SetUp() {
    person = Info::create(10);
    person->set(Params()
                .value("id", 42),
                1.0);
    person->set(Params()
                .value("azimut", Angle(0)),
                1.0);
    person->set(Params()
                .value("azimut", Angle(0)),
                1.0);
    person->set(Params()
                .value("azimut", Angle(0.1)),
                2.0);
    person->set(Params()
                .value("azimut", Angle(0.2))
                .value("id", 42),
                3.0);
    person->set(Params()
                .value("azimut", Angle(0.3)),
                4.0);
    person->set(Params()
                .value("id", 42),
                5.0);
  }
  Info::Ptr person;
};

TEST_F(StampedParams, GetMostRecent)
{
  EXPECT_EQ(42,  person->get<int>("id"));
  EXPECT_EQ(0.3, person->get<Angle>("azimut"));
}

TEST_F(StampedParams, GetAt)
{
  EXPECT_EQ(0.0, person->get<Angle>("azimut", 1.0));
  EXPECT_EQ(0.2, person->get<Angle>("azimut", 3.0));
}

TEST_F(StampedParams, GetInterpoled)
{
  EXPECT_DOUBLE_EQ(0.05, person->get<Angle>("azimut", 1.5));
  EXPECT_DOUBLE_EQ(0.09, person->get<Angle>("azimut", 1.9));
}

TEST_F(StampedParams, GetInterpoledFail)
{
  EXPECT_THROW(person->get<Angle>("azimut", 0.5), ParameterNotDefined);
  EXPECT_THROW(person->get<Angle>("azimut", 5.5), ParameterNotDefined);
}

TEST_F(StampedParams, Remove)
{
  EXPECT_TRUE(person->has("id"));
  person->remove("id", 3.5);
  // id have been removed at 3.5 but redefined at 5.0,
  // so it should be defined now
  EXPECT_TRUE(person->has("id"));
  person->remove("id", 5.5);
  EXPECT_FALSE(person->has("id"));
}

TEST_F(StampedParams, RemoveOnDefined)
{
  EXPECT_TRUE(person->has("id", 3.0));
  person->remove("id", 3.0);
  EXPECT_FALSE(person->has("id", 3.0));
}

/*
 * A param is removed when it is known to be false, else it would have
 * simply be updated. This means that we no longer want to use it for
 * interpolation.
 */
TEST_F(StampedParams, RemoveBreaksInterpolation)
{
  EXPECT_TRUE(person->has("azimut", 2.4));
  EXPECT_TRUE(person->has("azimut", 2.6));
  person->remove("azimut", 2.5);
  EXPECT_FALSE(person->has("azimut", 2.4));
  EXPECT_FALSE(person->has("azimut", 2.6));
  EXPECT_THROW(person->get<Angle>("azimut", 2.4), ParameterNotDefined);
  EXPECT_THROW(person->get<Angle>("azimut", 2.6), ParameterNotDefined);
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
