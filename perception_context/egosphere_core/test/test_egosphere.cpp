#include "common.hpp"

class EgosphereFixture : public testing::Test
{
 protected:
  void SetUp() {

    e.newInfo("battery, power, health", 0, Params()
      .value("level", 13)
      .value("charging", false));

    Info::Ptr room = e.newInfo("room", 0, Params()
      .value("name", std::string("kitchen")));

    e.associate(room, e.newInfo("power_socket, power", 0, Params()
      .value("free", true)
      .value("azimut", 12.5)));

    //e.associate(room, e.newInfo("table"));

    //socket.setParam("location", "");
  }
  Egos e;
};

/*
 * Unit tests
 */

TEST_F(EgosphereFixture, GetByTag_OneMatch)
{
  Info::Itr it = e.infoByTag("battery");
  EXPECT_TRUE(it);
  EXPECT_TRUE(it->is("power"));
  it++;
  EXPECT_FALSE(it);
}

TEST_F(EgosphereFixture, GetByTag_NoMatch)
{
  Info::Itr people = e.infoByTag("people");
  EXPECT_FALSE(people);
}

TEST_F(EgosphereFixture, GetByTag_TwoMatches)
{
  Info::Itr power = e.infoByTag("power");
  EXPECT_TRUE(power);
  power++;
  EXPECT_TRUE(power);
  power++;
  EXPECT_FALSE(power);
}

TEST_F(EgosphereFixture, GetInfoByAssociation)
{
  Info::Itr room = e.infoByTag("room");
  Info::Itr room_items = e.infoByAssociation(room);
  EXPECT_TRUE(room_items);
}

TEST_F(EgosphereFixture, InfoExpiration)
{
  Info::Itr room = e.infoByTag("room");
  ASSERT_TRUE(room);
  room->expire();
  room = e.infoByTag("room");
  EXPECT_FALSE(room);
  // Assocation must be removed too
  Info::Itr socket = e.infoByTag("power_socket");
  ASSERT_TRUE(socket);
  EXPECT_FALSE(e.infoByAssociation(socket));
}

TEST_F(EgosphereFixture, NoExpiredReferences)
{
  Info::Ptr room = e.infoByTag("room");
  ASSERT_TRUE(room);
  ASSERT_NE(1, room.use_count());
  room->expire();
  ASSERT_EQ(1, room.use_count());
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  //ros::Time::init();
  return RUN_ALL_TESTS();
}
