#ifndef _EGOSPHERE_ROS_PARAMS_HPP_
#define _EGOSPHERE_ROS_PARAMS_HPP_

#include <egosphere/parameter.hpp>

#include "egosphere_ros/MsgParameters.h"
#include "egosphere_ros/parameter_serializable.hpp"

namespace egosphere_ros {

/*
 * Standard parameters map, plus a flag to indicates
 * if thoose parameters have been published.
 */
class Params : public egosphere::ParametersMap<Params, std::string>
{
private:

  typedef egosphere::ParametersMap<Params, std::string> super;

public:

  // Tells wether or not this parameter have been published
  // published before (no need to publish it again, this happens
  // when the parameter is set from an other representation)
  Params & published(bool b);

  bool published() const;

  // When the user adds a parameter of some serializable type T, it is
  // stored in the representation with its serialization.
  template <class T>
  Params & value(const Key & key, const T & value)
  {
    ParameterAndItsSerialization param;
    param.unserialized(value);
    return super::value(key, param);
  }

  // When the users reads a parameter of some type T, it unserializes
  // the stored parameter if not already done, then returns the
  // unserialized parameter
  template <class T>
  const T & value(const Key & key)
  {
    return super::value<ParameterAndItsSerialization>(key).unserialized<T>();
  }

  // Fills a message from parameters (to notify other representations)
  void toMsg(MsgParameters & msg) const;

  // Fills parameters from a message (comming from another representation)
  void fromMsg(const MsgParameters & msg);

private:

  bool published_;

};

// Standard types serialization
//
// Param::values works with ParameterSerializable objects. That's fine
// for our Custom paramaters, but we need specialisations for each
// standard types.
//
// ie. this allows to call value<uint8_t>("some_param", 3), even if
// uint8_t is not a ParameterSerializable.

#define PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(T) \
  template <> \
  Params & Params::value(const Key & key, const T & value); \
  template <> \
  const T & Params::value(const Key & key);

PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(uint8_t);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION( int8_t);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(uint16_t);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION( int16_t);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(uint32_t);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION( int32_t);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(uint64_t);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION( int64_t);

PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(double);
PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(float);

PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(std::string);

// FIXME see implementation
//PARAMETER_DECLARE_STD_TYPE_SPECIALIZATION(bool);

}

#endif
