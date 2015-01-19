#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>

#include "egosphere_ros/params.hpp"

using namespace egosphere_ros;
using namespace std_msgs;

Params & Params::published(bool b)
{
  published_ = b;
  return *this;
}

bool Params::published() const
{
  return published_;
}

// Fills a message from parameters
void Params::toMsg(MsgParameters & msg) const
{
  typedef std::pair<std::string, egosphere::Parameter> Param;
  BOOST_FOREACH(const Param & param, values())
  {
    MsgParameter msg_param;
    msg_param.name  = param.first;
    // TODO possible to do without a copy ?
    msg_param.value = param.second.value<ParameterAndItsSerialization>().serialized();
    msg.params.push_back(msg_param);
  }
}

// Fills parameters from a message
void Params::fromMsg(const MsgParameters & msg)
{
  typedef std::pair<std::string, egosphere::Parameter> Param;
  BOOST_FOREACH(const MsgParameter & param, msg.params)
  {
    ParameterAndItsSerialization any;
    any.serialized(param.value);
    super::value(param.name, any);
  }
}

// Standard types specialisation

template<class StdType, class StdMsg>
class ParameterStdAsMsg : public ParameterAsMsg<StdMsg>
{
 public:

  const StdType & data() const {
    return this->msg_.data;
  }

  void data(const StdType & value) {
    this->msg_.data = value;
  }
};

#define PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(T,M) \
  template <> \
  Params & Params::value(const Key & key, const T & param) { \
    ParameterStdAsMsg<T,M> msg; \
    msg.data(param); \
    return value(key, msg); \
  } \
  template <> \
  const T & Params::value(const Key & key) { \
    return value<ParameterStdAsMsg<T,M> >(key).data(); \
  }

PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(uint8_t, UInt8);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION( int8_t,  Int8);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(uint16_t, UInt16);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION( int16_t,  Int16);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(uint32_t, UInt32);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION( int32_t,  Int32);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(uint64_t, UInt64);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION( int64_t,  Int64);

PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(float,  Float32);
PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(double, Float64);

PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(std::string, String);

// FIXME issues a warning (return a reference to temp)
//PARAMETER_IMPLEMENT_STD_TYPE_SPECIALIZATION(bool, Bool);
