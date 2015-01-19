#ifndef _EGOSPHERE_PARAMETER_HPP_
#define _EGOSPHERE_PARAMETER_HPP_

#include <boost/any.hpp>
#include <map>

namespace egosphere {

class ParameterNotDefined : std::exception {};

class Parameter
{
 public:

  Parameter() {}
  Parameter(const boost::any & value) : value_(value) {}

  template <typename T>
  const T& value() const
  {
    if (not defined()) {
      throw ParameterNotDefined();
    } else {
      return *boost::any_cast<T>(&value_);
    }
  }

  template <typename T>
  T& value()
  {
    if (not defined()) {
      throw ParameterNotDefined();
    } else {
      return *boost::any_cast<T>(&value_);
    }
  }

  void value(const boost::any & val)
  {
    value_ = val;
  }

  bool defined() const
  {
    return not value_.empty();
  }

  const std::type_info & type() const
  {
    return value_.type();
  }

  /*
  * returns an undefined parameter.
  */
  static Parameter undefined()
  {
    return Parameter();
  }

private:

  boost::any value_;
};

/*
* Derived to be able to keep the fluent interface with child classes
* (curiously recurring template pattern)
*/
template <typename Derived, typename Key_>
class ParametersMap
{
public:

  /* expose the key type for the egosphere */
  typedef Key_ Key;
  typedef typename std::map<Key, Parameter>::iterator iterator;
  typedef typename std::map<Key, Parameter>::const_iterator const_iterator;
  typedef std::pair<Key, Parameter> pair;

  /*
  * /!\ A ParameterMap can contain an undefined param. This intended,
  * use defines if you want to know if a param is defined.
  */
  bool contains(const Key & key) const
  {
    return params_.find(key) != params_.end();
  }

  bool defines(const Key & key) const
  {
    const_iterator i = params_.find(key);
    return i != params_.end() && i->second.defined();
  }

  Derived & value(const Key & key, const boost::any & value)
  {
    params_[key].value(value);
    return self();
  }

  // Non const to allow parameters with some have lazy loading mechanisms
  // ie. a ROS message that will be deserialized when accessed. It will
  // then store the result not to have to do it everytime.

  template <typename T>
  T & value(const Key & key)
  {
    iterator i = params_.find(key);
    if(i == params_.end())
      throw ParameterNotDefined();
    return i->second.value<T>();
  }

  /*
  * Removes a parameter.
  * (which is different than undefined)
  */
  void remove(const Key & key)
  {
    params_.erase(key);
  }

  /*
  * Undefine a parameter,which suggest that the parameter have been
  * defined, then undefined (remove doesn't allow to know if the
  * parameter have been previously defined).
  */
  Derived & undefine(const Key & key)
  {
    params_[key] = Parameter::undefined();
    return self();
  }

  void merge(const Derived & other)
  {
    for(
        const_iterator i = other.params_.begin();
        i != other.params_.end();
        i++
      ){
      params_[i->first] = i->second;
    }
  }

  const std::map<Key, Parameter> & values() const
  {
    return params_;
  }

private:

  Derived & self()
  {
    return static_cast<Derived &>(*this);
  }

  std::map<Key, Parameter> params_;
};

/* TODO fuse with multiple parameters */

/*
* Returns a new parameter being the fusion of this one
* and an other. In most of the cases fusion
* of different types parameters will fail.
*/
//	virtual ParameterPtr fuseWith(ParameterConstPtr) const = 0;

// will allow things like setParam<Location>("key", loc)

/*	Parameter fuseWith(const Parameter<T>& p) const
    {
// FIXME stub implementation
return ParameterPtr(new Parameter<T>((value_ + p.value_)/2));
}

Parameter fuseWith(ParameterConstPtr param) const
{
if(typeid(ParameterStd<T>) == typeid(*param))
{
return fuseWith(dynamic_cast<const ParameterStd<T>&>(*param));
} else {
throw NotImplemented();
}
}*/

}

#endif
