#ifndef _EGOSPHERE_INFORMATION_HPP_
#define _EGOSPHERE_INFORMATION_HPP_

#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/utility.hpp>
#include <boost/signals.hpp>

#include "parameter.hpp"
#include "tags.hpp"
#include "iterator.hpp"

#include <iostream>

namespace egosphere {

/*
 * Non copyable to force the use of pointer
 * (the boost::signal member is non copyable too)
 *
 * http://www.boost.org/doc/libs/1_47_0/libs/iterator/doc/indirect_iterator.html
 * see Type Erasure for C++ Iterators
 * http://www.jezuk.co.uk/cgi-bin/view/jez?id=3612
 *
 * TODO expiration in time, Information responsability?
 * TODO check if cache max duration is useful
 *
 * Time() should always return the same value (ie. not the current time) if
 * you want to be able to use functions without time (ie set(val)
 * instead of set(val, t).
 *
 * Time - Time must be a Duration
 */

template<typename Derived, typename Params_, typename Time_, typename Duration_>
class Information /*: boost::noncopyable*/
{
 public:

  /* Egosphere needs to know the templated types, so we expose them */
  typedef Params_   Params;
  typedef Time_     Time;
  typedef Duration_ Duration;
  typedef boost::shared_ptr<Derived> Ptr;
  typedef PointerIterator<Derived, const Ptr> Itr;
  typedef uint64_t UpId; // incremented at each update

  /* Used internally to avoid bloated syntax */
  typedef typename Params::Key ParamsKey;
  typedef typename std::map<ParamsKey, Parameter> ParamsMap;
  typedef std::map<Time, std::pair<Params, UpId>, std::greater<Time> > map;
  typedef typename map::const_iterator map_const_itr;
  typedef typename map::iterator map_itr;

  // Non copyable, force to use the create method and to manipulate
  // information pointers only. Required not to duplicate callbacks
  // on updates for exemple.

protected:
  Information() :
    upid_(0),
    cache_size_(1),
    expired_(false)
  {}

  // Protected non-virtual destructor, TODO check if that's ok
  ~Information () {}

private:
    Information (const Information &);
    Information & operator = (const Information &);

public:

  /*
   * The user have to use this method to create
   * an Info instance
   *
   * cache_duration : duration of the short term memory
   * cache_size     : max elements in the short term memory (0 = no limit)
   */
  static Ptr create(Duration cache_duration, unsigned int cache_size = 1000)
  {
    Ptr p(new Derived);
    p->shared_this_ = p;
    p->cacheLimits(cache_duration, cache_size);
    return p;
  }

  void cacheLimits(const Duration & d, unsigned int size)
  {
    cache_duration_ = d;
    cache_size_     = size;
    cleanParams();
  }

  void cacheSizeLimit(unsigned int size)
  {
    cache_size_ = size;
    cleanParams();
  }

  void cacheDurationLimit(const Duration & d)
  {
    cache_duration_ = d;
    cleanParams();
  }

  unsigned int cacheSizeLimit() const
  {
    return cache_size_;
  }

  const Duration & cacheDurationLimit() const
  {
    return cache_duration_;
  }

  bool is(const std::string & tag)
  {
    return tags_.contains(tag);
  }

  const Tags & tags() const
  {
    return tags_;
  }

  /*
   * Tried with Tags & tags, but apprently c++ does't want us
   * to modify temporaries (ie. set(Tags("test,try")) will not work)
   */
  const Ptr & set(Tags tags)
  {
    tags_.splice(tags);
    // TODO allow to change tags after creation ?
    //doUpdate();
    return shared_this_;
  }

  const map & params() const
  {
    return params_;
  }

  // TODO Remove this one when not needed anymore
  // (the user should not play with this)
  map & params()
  {
    return params_;
  }

  bool has(const ParamsKey & param) const
  {
    if(params_.begin() != params_.end())
    {
      return has(param, params_.begin()->first);
    } else {
      return false;
    }
  }

  /*
   * Tells if a  parameter can be read/interpoled at some time.
   * (ie. it have been defined at t0 <= t and t1 > t
   */
  bool has(const ParamsKey & name, Time t) const
  {
    std::pair<map_const_itr, map_const_itr> i =
        boundingParams(name, t);
    map_const_itr & i0 = i.first, i1 = i.second;
    // If no lower bound have been found, param is not defined
    if (i0 == params_.end()) {
      return false;
    }
    // If param contains name at t
    if (i0->first == t) {
      return i0->second.first.defines(name);
    }
    // Else we need to interpolate, if no upper bound have been
    // found, param is not interpolable.
    return (i1 != params_.end()) &&
        i0->second.first.defines(name) &&
        i1->second.first.defines(name);
  }

  /*
   * Set parameters with no time specified, not recommened,
   * used for tests. May need to be overloaded for some Time classes
   * (ie. for ros::Time, using ros::Time::now can be useful)
   */
  const Ptr & set(const Params & params)
  {
    return set(params, Time());
  }

  /*
   * TODO splice, like tags to avoid copies
   * TODO insert(guess, params) to speedup
   */
  const Ptr & set(const Params & params, const Time & t)
  {
    // Keeps track of the most recent values of each parameter name
    for (
        typename ParamsMap::const_iterator i = params.values().begin();
        i != params.values().end();
        i++
    ) {
      const ParamsKey & name = i->first;
      if (
          most_recent_value_.find(name) == most_recent_value_.end()
          || most_recent_value_[name] < t
      ) {
        most_recent_value_[name] = t;
      }
    }
    for(map_itr i = params_.begin(); i != params_.end(); i++)
    {
      // If params have already been defined at this timestamp,
      // merge the new ones with the old ones
      if(i->first == t) {
        i->second.first.merge(params);
        i->second.second = upid_;
        doUpdate();
        return shared_this_;
      }
      // If there is no params defined at this timestamp,
      // insert them
      if(i->first < t) {
        break;
      }
    }
    params_.insert(std::make_pair(t, std::make_pair(params, upid_)));
    doUpdate();
    return shared_this_;
  }

  /*
   * Returns the most recent value of param named name
   *
   * TODO there is currently no way for the user to retreive
   * value stamp
   */
  template <typename T>
  const T & get(const ParamsKey & name)
  {
    for(map_itr i = params_.begin(); i != params_.end(); i++)
    {
      if (i->second.first.contains(name)) {
        return i->second.first.template value<T>(name);
      }
    }
    throw ParameterNotDefined();
  }

  const Time & getTimeStamp(const ParamsKey & name)
  {
    for(map_itr i = params_.begin(); i != params_.end(); i++)
    {
      if (i->second.first.contains(name)) {
        return i->first;
      }
    }
    throw ParameterNotDefined();
  }

  template<typename T>
  const T get(const ParamsKey & name, const Time & t)
  {
    std::pair<map_itr, map_itr> i =
        boundingParams(name, t);
    map_itr & i0 = i.first, i1 = i.second;
    // If no lower bound have been found, param is not defined
    if (i0 == params_.end()) {
      throw ParameterNotDefined();
    }
    // If param is defined at t, returns its value without interpolation
    if (i0->first == t) {
      return i0->second.first.template value<T>(name);
    }
    // Else we need to interpolate, if no upper bound have been
    // found, param is not interpolable.
    if (i1 == params_.end()) {
      throw ParameterNotDefined();
    }
    Duration t0_t  = t - i0->first;
    Duration t0_t1 = i1->first - i0->first;
    return interpolate(
        i0->second.first.template value<T>(name),
        i1->second.first.template value<T>(name),
        t0_t / t0_t1);
  }

  Time lastUpdate(const ParamsKey & name) const
  {
    for(map_const_itr i = params_.begin(); i != params_.end(); i++)
    {
      if (i->second.first.contains(name)) {
        return i->first;
      }
    }
    throw ParameterNotDefined();
  }

  /*
  * Udefined is used here, allowing the 'has' function to figure out that
  * the parameter have been undefied at that time (not being present would
  * have meant not updated at this time).
  */
  void remove(const ParamsKey & name, const Time & t)
  {
    set(Params().undefine(name), t);
  }

  void remove(const ParamsKey & name)
  {
    remove(name, Time());
  }

  // 1 if most recent update is t
  // 0 if it is older than cache_duration
  float freshness(const Time & now) const
  {
    if (params_.size() == 0) {
      return 0.0;
    }
    const Time & last_up = params_.begin()->first;
    if (last_up < now - cache_duration_) {
      return 0.0;
    }
    if(last_up > now) {
      return 1.0;
    }
    return (last_up - (now - cache_duration_)) / cache_duration_;
  }

  bool expired() const
  {
    return expired_; // TODO OR now > laststamp + duration
  }

  void expire()
  {
    if (not expired_)
    {
      expired_ = true;
      doUpdate();
    }
    shared_this_.reset();
  }

  const UpId & lastUpId() const
  {
    return upid_;
  }

  void onUpdate(boost::function<void (const Ptr &)> f)
  {
    update_notifier_.connect(f);
  }

  /*
  * Finds the most recent time at which every given information
  * can tell its parameters.
  *
  * FIXME it is possible that we can't get the parameter for every
  *       information at t.
  *       ie. info1[param] have been set defined t=1
  *           info2[param] have been set defined t=2 for the first time
  *           => the function will return 1,
  *              but info2[param] is not defined at this time
  *
  *      in this case, is it worth searching if we can find an earlier
  *      update time where every info can tell it's param value, or
  *      a ParamNotDefined exception is fine?
  */
  static Time latestCommonUpdateStamp(
      Itr & info, const ParamsKey param
      ) {
    if (not info) { throw ParameterNotDefined(); }
    Time t = info->lastUpdate(param);
    info++;
    while(info) {
      Time t1 = info->lastUpdate(param);
      if(t1 < t) { t = t1; }
      info++;
    }
    return t;
  }

  // TODO notify when deleted
  // expired param ?

private:

  // tells if we can flush the parameters defined at t
  // (we can't if this is their latest defined value)
  template<class Itr>
  bool flushable(const Itr & itr)
  {
    bool ok = true;
    const Time & t = itr->first;
    const ParamsMap & params = itr->second.first.values();
    for (
        typename ParamsMap::const_iterator i = params.begin();
        i != params.end();
        i++
    ) {
      if (most_recent_value_[i->first] == t) {
        ok = false;
        break;
      }
    }
    return ok;
  }

  void cleanParams()
  {
    assert(cache_size_ > 0);
    if (expired()) {
      params_.clear();
      return;
    }
    if (params_.size() <= 1) {
      return;
    }

    typename map::iterator flush_me = --params_.end();

    while (
        flush_me != params_.begin()
        && not (
          (params_.size() <= cache_size_) &&
          (params_.begin()->first - flush_me->first <= cache_duration_))
    ){
      if (flushable(flush_me)) {
        params_.erase(flush_me--);
      } else {
        flush_me--;
      }
    }
  }

  void doUpdate()
  {
    cleanParams();
    update_notifier_(shared_this_);
    upid_++;
  }

  /*
  * returns a pair of iterators i0 and i1 representing params
  * set at t0 immediately <= t and t1 immediately > t, containing
  * a value for name.
  *
  * /!\ Containing a value doesn't mean that the value is defined
  *     ie. params containing {name1 => val1, name2 => undef}, can
  *     be returned for name2. This is intended and allows to make
  *     the difference between a removed parameter, and a not updated
  *     parameter.
  */
  std::pair<map_itr, map_itr> boundingParams(
      const ParamsKey & name,
      const Time & t
  ){
    map_itr i0, i1 = params_.end();
    for(i0 = params_.begin(); i0 != params_.end(); i0++)
    {
      if (i0->second.first.contains(name)) {
        // while param stamp > than t,
        // set its value to t1 (first immediately more recent than t)
        if (i0->first > t) {
          i1 = i0;
          // When params stamps is no longer > than t,
          // stop the loop and return an interpoled value
          // of the param at t.
        } else {
          break;
        }
      }
    }
    return std::make_pair(i0, i1);
  }

  // TODO Don't know how a function returning itr or const itr
  // sould be implemented.

  std::pair<map_const_itr, map_const_itr> boundingParams(
      const ParamsKey & name,
      const Time & t
  ) const {
    map_const_itr i0, i1 = params_.end();
    for(i0 = params_.begin(); i0 != params_.end(); i0++)
    {
      if (i0->second.first.contains(name)) {
        // while param stamp > than t,
        // set its value to t1 (first immediately more recent than t)
        if (i0->first > t) {
          i1 = i0;
          // When params stamps is no longer > than t,
          // stop the loop and return an interpoled value
          // of the param at t.
        } else {
          break;
        }
      }
    }
    return std::make_pair(i0, i1);
  }


  Tags tags_;

  UpId upid_;
  map params_;

  // contains the most recent version of a param
  // used in cleanParams to keep the param in the cache
  // if it is the last known value (even if it is old)
  std::map<ParamsKey, Time> most_recent_value_;

  Duration cache_duration_;
  unsigned int cache_size_;

  Ptr shared_this_;

  bool expired_;
  boost::signal<void (const Ptr &)> update_notifier_;
};


}

#endif
