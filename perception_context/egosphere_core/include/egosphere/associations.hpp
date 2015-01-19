#ifndef _EGOSPHERE_ASSOCIATION_HPP_
#define _EGOSPHERE_ASSOCIATION_HPP_

#include <utility>
#include <set>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/iterator/transform_iterator.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include "iterator.hpp"

// TODO transform_iterator to return only the other end
// of the link

/*
 * TODO absolutely not optimised, find a better way of doing this
 *
 * Bi-directional many to many association between objects
 *
 * Intended usage :
 *  - associate legs, person (= associate person, legs)
 *  - dissociate legs, person
 *  - get person by legs |
 *  - get legs by person +-> through user filter
 *  - get all by person
 *
 */

namespace egosphere {

template <typename T>
class Associations
{
 public:

  /*
   * A pair of 2 associated elements
   */
  typedef typename std::pair<T,T> pair;

  class iterator; // defined at en of the class

  /* Mainly for BOOST_FOREACH */
  typedef std::pair<iterator, iterator> iterators;

  /*
   * As we want association to be bidirectional, we don't
   * want to have a difference between a->b and b->a. By
   * convention, the tiniest element is set in first.
   *
   * FIXME allow association a->a ?
   */
  static pair make_pair(const T & a, const T & b)
  {
    if (a < b) {
      return pair(a,b);
    } else {
      return pair(b,a);
    }
  }

  void associate(const T & a, const T & b)
  {
    pairs_.insert(make_pair(a, b));
  }

  void dissociate(const T & a, const T & b)
  {
    pairs_.erase(pairs_.find(make_pair(a,b)));
  }

  void erase(const T & a)
  {
    typename std::set<pair>::iterator it = pairs_.begin();
    while (it != pairs_.end()) {
      if (it->first == a || it->second == a) {
        pairs_.erase(it++);
      } else {
        it++;
      }
    }

  }

  const std::set<pair> & pairs() const
  {
    return pairs_;
  }

  /*
   * Tells if the too parameters are associated
   */
  bool associated(const T & a, const T & b) const
  {
    return pairs_.find(make_pair(a,b)) != pairs_.end();
  }

  iterator begin(
      const T & pairedWith,
      const boost::function<bool(const T &)> & filter = NULL
      ) {
    return iterator(pairs_.begin(), pairs_.end(), pairedWith, filter);
  }

  iterator end(
      const T & pairedWith,
      const boost::function<bool(const T &)> & filter = NULL
      ) {
    return iterator(pairs_.end(), pairs_.end(), pairedWith, filter);
  }

  iterators each(
      const T & pairedWith,
      const boost::function<bool(const T &)> & filter = NULL
      ) {
    return iterators(
        begin(pairedWith, filter),
        end(pairedWith, filter)
        );
  }

  /*
   * Used to iterate through associations.
   * ie. Association<int>::iterator(i1, i2, 7) will iterate through all
   * pairs containing 7, deference returns the int associated with 7
   *
   * Tried to do this with filter_iterator + transform_iterator,
   * but it was a mess.
   *
   * TODO make it bidirectional
   * TODO allow removals of associations from iterator
   *
   */

  class iterator : public boost::iterator_facade<
                   iterator,
                   pair const,
                   boost::forward_traversal_tag,
                   const T &
                   >{

                    public:

                     /*
                      * We need to provide the end of the set, to know when to stop
                      * when filtering. This alows us to define the bool() operator :
                      * iterator == true : not the end, iterator == false : the end.
                      */
                     explicit iterator(
                         const typename std::set<pair>::const_iterator & it,
                         const typename std::set<pair>::const_iterator & end,
                         const T& pairedWith,
                         const boost::function<bool(const T &)> & f = NULL
                         ) :
                         it_(it), end_(end),
                         pairedWith_(pairedWith),
                         userFilter_(f)
                     {
                       apply_filters();
                     }

                     /*
                      * TODO see if this is already defined by boost
                      */
                     operator bool() const
                     {
                       return it_ != end_;
                     }

                    private:

                     friend class boost::iterator_core_access;

                     // Jumps over every pair not containing pairedWith_
                     void apply_filters()
                     {
                       while(it_ != end_)
                       {
                         if (it_->first == pairedWith_ || it_->second == pairedWith_)
                         {
                           if(!userFilter_) {
                             break;
                           }
                           if(it_->first != pairedWith_ && userFilter_(it_->first)) {
                             break;
                           }
                           if(it_->second != pairedWith_ && userFilter_(it_->second)) {
                             break;
                           }
                         }
                         it_++;
                       }
                     }

                     const T & dereference() const {
                       if (it_->first != pairedWith_) {
                         return it_->first;
                       } else {
                         return it_->second;
                       }
                     }

                     void increment() { it_++; apply_filters(); }
                     bool equal(const iterator & other) const{
                       return it_ == other.it_;
                     }

                     typename std::set<pair>::const_iterator it_;
                     typename std::set<pair>::const_iterator end_;
                     T pairedWith_;
                     boost::function<bool(const T &)> userFilter_;
                   };

 private:

  std::set<pair> pairs_;

};

}

#endif
