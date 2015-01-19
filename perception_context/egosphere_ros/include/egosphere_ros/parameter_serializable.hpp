#ifndef _EGOSPHERE_ROS_PARAMETER_SERIALIZABLE_HPP_
#define _EGOSPHERE_ROS_PARAMETER_SERIALIZABLE_HPP_

#include <egosphere/parameter.hpp>
#include <boost/any.hpp>
#include <boost/foreach.hpp>

namespace egosphere_ros {

typedef std::vector<uint8_t> ParameterSerialized;

// Custom parameters types must respect this interface
// (There is currently no enforcement on this, but thoose function
// are used by templated methods)
//
// They must be fully initializable like this
//   ParameterSerializable p;
//   p.unserialize(...);
//
// Custom parameters must be serializable to send them between nodes

class ParameterSerializable
{
 public:
  virtual ParameterSerialized serialize() const = 0;
  virtual void unserialize(const ParameterSerialized &) = 0;
};

// A parameter serializable represented as a ROS message
// This is the advised way to go for Custom parameters
// to benefit from the nice ROS serialization.
//
// see egosphere_std_params for examples

class WrongSerialization : std::exception {};

template<class Msg>
class ParameterAsMsg : public ParameterSerializable
{
 public:

  void unserialize(const ParameterSerialized & serialized)
  {
    namespace ser = ros::serialization;
    // FIXME constness broken
    ser::IStream stream((uint8_t*)&serialized[0], serialized.size());
    try {
      ser::Serializer<Msg>::read(stream, msg_);
    }
    // Serialized is too short to contain this type of message
    catch (ser::StreamOverrunException e)
    {
      throw WrongSerialization();
    }
    // Serialized is too long to contain this type of message
    if (stream.getLength() != 0) {
      throw WrongSerialization();
    }
  }

  ParameterSerialized serialize() const
  {
    ParameterSerialized serialized;
    namespace ser = ros::serialization;
    uint32_t size = ser::serializationLength(msg_);
    serialized.resize(size);
    ser::OStream stream(&serialized[0], size);
    ser::serialize(stream, msg_);
    return serialized;
  }

 protected:

  Msg msg_;

};

// An object containing a serializable object with its serialization.
//
// May be initialized with a serialized parameter.
//   ie. when receiving a ROS message. At this
//   point we don't know the type, so the message
//   will be unserialized when asked by the user
//
// May be initialized with an unserialized parameter.
//   ie. when the user adds an information in the
//   internal representation. The serialization will be
//   required later when the object type will no longer be
//   available (when sending a ROS message to the other
//   representations).
//
// Methods are templated, but they are made
// to handle instances of ParameterSerializable.
// TODO Find a way to enforce this

class ParameterAndItsSerialization
{
 public:

  // Constructs the parameter from a serializable
  // object. We know its type at this time, so we serialize it
  // directly.
  template <class Serializable>
  void unserialized(const Serializable & param)
  {
    unserialized_ = boost::any(param);
    serialized_   = param.serialize();
  }

  // Constructs the parameter from serialization. We don't
  // know the deserialized type at this point.
  void serialized(const ParameterSerialized & param)
  {
    serialized_ = param;
    unserialized_ = boost::any();
  }

  template <class Serializable>
  const Serializable & unserialized()
  {
    // When constructed by serialization, we have not
    // been able to deserialize before
    if (unserialized_.empty())
    {
      assert(!serialized_.empty());
      Serializable param;
      param.unserialize(serialized_);
      unserialized_ = boost::any(param);
    }

    Serializable * ptr = boost::any_cast<Serializable>(&unserialized_);
    if(ptr == NULL) {
      throw boost::bad_any_cast();
    } else {
      return *ptr;
    }
  }

  const ParameterSerialized & serialized() const
  {
    return serialized_;
  }

 protected :

  boost::any unserialized_;
  ParameterSerialized serialized_;
};

}

#endif
