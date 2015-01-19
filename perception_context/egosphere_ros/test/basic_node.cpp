#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "egosphere_ros/egosphere.hpp"
#include <std_msgs/UInt32.h>

using namespace egosphere_ros;

// TODO
//  . check strange initialization (got problems launching and relaunching
//  lots of time the tests with a unique master)

void spin()
{
  do {
    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
  }  while (not ros::getGlobalCallbackQueue()->isEmpty());
}

TEST(Egosphere, UniqId)
{
  ros::NodeHandle n("~");
  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");
  EXPECT_NE(e1.uid(), e2.uid());
}

TEST(Egosphere, NewInfoReceivedNoParams)
{
  ros::NodeHandle n("~");

  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");

  e1.newInfo("battery, power, health", Params());

  EXPECT_TRUE(e1.infoByTag("battery"));
  EXPECT_FALSE(e2.infoByTag("battery"));

  spin();

  EXPECT_TRUE(e1.infoByTag("battery"));
  EXPECT_TRUE(e2.infoByTag("battery"));
}

TEST(Egosphere, NewInfoReceivedParams)
{
  ros::NodeHandle n("~");

  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");

  e1.newInfo("battery, power, health", Params()
    .value("level", (uint8_t) 13)
    .value("charging", (uint8_t) false)
  );

  spin();

  ASSERT_TRUE(e2.infoByTag("battery"));

  Info::Ptr info = e2.infoByTag("battery");
  EXPECT_TRUE(info->has("charging"));
  ASSERT_TRUE(info->has("level"));
  EXPECT_EQ(13, info->get<uint8_t>("level"));
}

TEST(Egosphere, NewParamsReceived)
{
  ros::NodeHandle n("~");

  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");

  Info::Ptr battery = e1.newInfo("battery, power, health", Params()
    .value("level", (uint8_t) 13)
    .value("charging", (uint8_t) false));

  battery->set(Params().value("level", (uint8_t) 12));

  spin();

  Info::Ptr i1 = e1.infoByTag("battery");
  Info::Ptr i2 = e2.infoByTag("battery");

  EXPECT_TRUE(i1 && i2);

  ASSERT_TRUE(i1->has("level"));
  EXPECT_EQ(12, i1->get<uint8_t>("level"));

  ASSERT_TRUE(i2->has("level"));
  EXPECT_EQ(12, i2->get<uint8_t>("level"));
}

TEST(Egosphere, InfoCacheSizeLocal)
{
  ros::NodeHandle n("~");

  Egosphere e(n, "/egosphere");

  Info::Ptr info = e.newInfo("sinus", Params()
    .value("v", (double) 0)
  );

  // Does not allow to memorise more than 5 info value.
  info->cacheSizeLimit(5);

  for(double x = 0; x < 6 * M_PI; x += 0.05)
  {
    info->set(Params().value("v", (double) sin(x)));
    EXPECT_EQ(sin(x), info->get<double>("v"));
    EXPECT_TRUE(info->params().size() <= 5);
  }

  spin();
}

TEST(Egosphere, InfoCacheSizeRemote)
{
  ros::NodeHandle n("~");

  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");

  Info::Ptr info = e1.newInfo("sinus", Params()
    .value("v", (double) 0)
  );

  spin();

  ASSERT_TRUE(e1.infoItr() && e2.infoItr());
  Info::Ptr i1 = e1.infoItr();
  Info::Ptr i2 = e2.infoItr();

  i1->cacheSizeLimit(5);
  i2->cacheSizeLimit(5);

  for(double x = 0; x < 6 * M_PI; x += 0.05)
  {
    info->set(Params().value("v", (double) sin(x)));

    spin();

    EXPECT_TRUE(i1->params().size() <= 5);
    EXPECT_TRUE(i2->params().size() <= 5);

    EXPECT_EQ(i1->lastUpdate("v"), i2->lastUpdate("v"));

    EXPECT_EQ(sin(x), i1->get<double>("v"));
    EXPECT_EQ(sin(x), i2->get<double>("v"));

    /*if(sin(x) != i2->get<double>("v"))
    {
      std::cout << "Plantage at " << x << std::endl;
    }

    std::cout << "i2\n";
    for(
      Info::map_itr it = i2->params().begin();
      it != i2->params().end();
      ++it
    ){
      std::cout << it->first << " " << it->second.first.value<double>("v") << std::endl;
    }*/
  }
}

TEST(Egosphere, InfoExpirationRemote)
{
  ros::NodeHandle n("~");

  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");

  Info::Ptr info = e1.newInfo("sinus", Params()
    .value("v", (double) 0)
  );

  spin();
  ASSERT_TRUE(e1.infoItr() && e2.infoItr());
  Info::Ptr i1 = e1.infoItr();
  Info::Ptr i2 = e2.infoItr();

  i1->expire();

  spin();
  EXPECT_TRUE(i2->expired());

}

TEST(Egosphere, NoReferencesToExpired)
{
  ros::NodeHandle n("~");

  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");

  Info::Ptr info = e1.newInfo("sinus", Params()
    .value("v", (double) 0)
  );

  spin();
  ASSERT_TRUE(e1.infoItr() && e2.infoItr());

  Info::Ptr i1 = e1.infoItr();
  Info::Ptr i2 = e2.infoItr();

  EXPECT_NE(1, i1.use_count());
  EXPECT_NE(1, i2.use_count());

  i1->expire();
  spin();

  // FIXME some references are remaining,
  // so expired informations remain in memory...
  //
  // EXPECT_EQ(1, i1.use_count());

  EXPECT_EQ(1, i2.use_count());
}

TEST(Egosphere, BackAndForth)
{
  ros::NodeHandle n("~");

  Egosphere e1(n, "/egosphere");
  Egosphere e2(n, "/egosphere");

  Info::Ptr info = e1.newInfo("sinus", Params()
    .value("v", (double) 0)
  );

  spin();
  ASSERT_TRUE(e1.infoItr() && e2.infoItr());
  Info::Ptr i1 = e1.infoItr();
  Info::Ptr i2 = e2.infoItr();

  i1->set(Params()
    .value("v", (double) 1.0));
  spin();

  EXPECT_EQ((double) 1.0, i2->get<double>("v"));

  i2->set(Params()
    .value("v", (double) 2.0));
  spin();

  EXPECT_EQ((double) 2.0, i1->get<double>("v"));
}

// TODO Test how many references to info remaining after removal

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_basic_node");
  int r = RUN_ALL_TESTS();
  spin();
  return r;
}
