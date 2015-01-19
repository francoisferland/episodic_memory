#include <gtest/gtest.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/String.h>
#include <egosphere_ros/parameter_serializable.hpp>

using namespace egosphere_ros;
using namespace std_msgs;

// Serialization with a ROS message,
// the advised way to go

class Color : public ParameterAsMsg<ColorRGBA>
{
 public :

  void set(float r, float g, float b, float a) {
    msg_.r = r;
    msg_.g = g;
    msg_.b = b;
    msg_.a = a;
  }

  bool operator== (const Color & other) const
  {
    return (msg_.r == other.msg_.r)
        && (msg_.g == other.msg_.g)
        && (msg_.b == other.msg_.b)
        && (msg_.a == other.msg_.a);
  }
};

class Text : public ParameterAsMsg<String>
{
 public:
  void set(const std::string & s)
  {
    msg_.data = s;
  }
  const std::string & get() const
  {
    return msg_.data;
  }
};

TEST(ParameterAsMsg, Unserialize)
{
  Color c1, c2;
  c1.set(1, 1, 0, 1);

  ParameterSerialized ser = c1.serialize();
  ASSERT_FALSE(ser.empty());

  c2.unserialize(ser);
  EXPECT_EQ(c1, c2);
}

// Testing with string because we can't know the
// serialized message size by advance (a string
// can be of any size, unlike an uint8_t for example)

TEST(ParameterAsMsg, UnserializeNonFixedSize)
{
  Text t1, t2;
  t1.set("some text");
  t2.unserialize(t1.serialize());
  EXPECT_EQ(t1.get(), t2.get());
}

// Warning, if by any change the serialization is wrong,
// but has the same length as a good one, nothing will
// be detected

TEST(ParameterAsMsg, UnserializeTooShortSerialization)
{
  Color c;
  std::string str = "random stuff";
  EXPECT_THROW(
    c.unserialize(ParameterSerialized()),
    WrongSerialization
  );
  EXPECT_THROW(
    c.unserialize(ParameterSerialized(str.begin(), str.end())),
    WrongSerialization
  );
}

TEST(ParameterAsMsg, UnserializeTooLongSerialization)
{
  Color c;
  std::string str = "random stuff bigger than a serialized color";
  EXPECT_THROW(
    c.unserialize(ParameterSerialized(str.begin(), str.end())),
    WrongSerialization
  );
}


TEST(ParameterAndItsSerialization, Init)
{
  Color c;
  ParameterAndItsSerialization p1, p2;
  c.set(1, 1, 0, 1);

  p1.unserialized(c);
  EXPECT_EQ(c, p1.unserialized<Color>());

  p2.serialized(p1.serialized());
  EXPECT_EQ(c, p2.unserialized<Color>());
}

TEST(ParameterAndItsSerialization, UnserializeOnce)
{
  Color c;
  ParameterAndItsSerialization p;
  c.set(1, 1, 0, 1);

  p.serialized(c.serialize());
  EXPECT_EQ(&p.unserialized<Color>(), &p.unserialized<Color>());
}

TEST(ParameterAndItsSerialization, WrongTypeUnserialize)
{
  Color c;
  ParameterAndItsSerialization p;

  p.serialized(c.serialize());

  EXPECT_THROW(
    p.unserialized<Text>(),
    WrongSerialization
  );
}

TEST(ParameterAndItsSerialization, WrongWithoutSerialize)
{
  Color c;
  ParameterAndItsSerialization p;

  p.unserialized(c);

  EXPECT_THROW(
    p.unserialized<Text>(),
    boost::bad_any_cast
  );
}

int main(int argc, char *argv[])
{
  testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
