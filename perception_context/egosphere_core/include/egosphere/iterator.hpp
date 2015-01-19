#ifndef _EGOSPHERE_ITERATOR_HPP_
#define _EGOSPHERE_ITERATOR_HPP_

#include <boost/shared_ptr.hpp>

namespace egosphere {

/*
 * An attempt to have a common interface for any forward iterator derefrenced
 * into the same type (type erasure iterator).
 *
 * Inpired by "A filtering iterator in C++", JezUK
 * http://www.jezuk.co.uk/cgi-bin/view/jez?id=3612
 *
 * Additionally, for easier use in this application, each Iterator contains
 * its end, knowing if it is valid or not.
 *
 * Notes
 *
 *   No equal operator between iterators, ok ?
 *   Performance implication of this ?
 *
 */

template<class V>
class Iterator
{
	/*
	 * Iterator holds any iterators implementing this interface
	 */
	class ItrConcept
	{
		public:

		virtual ~ItrConcept() {}
		// Tells if iterator points to something (ie. != end)
		virtual operator bool() const = 0;
		// Dereference
		virtual V & operator *() const = 0;
		// Move forward
		virtual void operator++(int) = 0;
		// To allow copy of Iterator
		virtual ItrConcept * clone() const = 0;
	};

	/*
	 * Concrete iterator from two standard iterators, ie. which can't tell if
	 * they actually point to something (like STL iterators)
	 */
	template< typename Tb, typename Te > class ItrModelStd : public ItrConcept
	{
		public:

		ItrModelStd(const Tb& it, const Te& end) : it_(it), end_(end) {}

		operator bool() const { return it_ != end_; }

		V & operator *() const { return *it_; }

		void operator++(int) { if(it_ != end_) it_++; }

		ItrModelStd<Tb, Te> * clone() const {
			return new ItrModelStd(it_, end_);
		}

		private:

		Tb it_;
		Te end_;
	};

	/*
	 * Concrete iterator from a "range", ie. an iterator which can tell if
	 * it is at the end by itself.
	 */
	template< typename T > class ItrModelRange : public ItrConcept
	{
		public:

		ItrModelRange(const T& it) : it_(it) {}

		operator bool() const {return bool(it_); }

		V & operator *() const {return *it_; }

		void operator++(int) { it_++; }

		ItrModelRange<T> * clone() const {
			return new ItrModelRange(it_);
		}

		private:

		T it_;
	};

	boost::shared_ptr<ItrConcept> itp_;

	public:

	template< typename T > Iterator(const T& it) :
		itp_(new ItrModelRange<T>(it)) {}

	template< typename Tb, typename Te> Iterator(const Tb& it, const Te& end) :
		itp_(new ItrModelStd<Tb, Te>(it, end)) {}

	Iterator(const Iterator& it) :
		itp_(it.itp_->clone()) {}

	operator bool() const {
		return bool(*itp_);
	}

	V & operator *() const {
		return *(*itp_);
	}

	V * operator->() const {
		return &(**this);
	}

	Iterator<V> & operator++(int) {
		(*itp_)++;
		return *this;
	}

	unsigned int count() const {
		Iterator it = *this;
		unsigned int c;
		for(c = 0; it; it++, c++);
		return c;
	}
};

/*
 * Same as Iterator, intended to be used when iterating over pointers.
 *
 * The goal is to allow using the iterator the pointer it is representing.
 *
 *   - it->method() will be the same that ptr->method()
 *   - it knows how to convert itself in a ptr
 *
 * Notes
 *
 *   V  is the type of the pointed data
 *   Vp is the type of the pointer (usually V* or shared_ptr<V>)
 *   bool(it) tells if it is in range, not that it is a valid ptr
 *
 */

template<class V, class Vp = V*>
class PointerIterator : protected Iterator<Vp>
{
public:
	template< typename T > PointerIterator(const T& it) :
		Iterator<Vp>(it) {}

	template< typename Tb, typename Te > PointerIterator(const Tb& it, const Te& end) :
		Iterator<Vp>(it, end) {}

	PointerIterator(const PointerIterator& it) :
		Iterator<Vp>(*static_cast<const Iterator<Vp>*>(&it)) {}

	operator bool() const {
		return * static_cast<const Iterator<Vp>*>(this);
	}

	V & operator *() const {
		return **(* static_cast<const Iterator<Vp>*>(this));
	}

	V * operator->() const {
		return &(**this);
	}
	
	PointerIterator<V,Vp> & operator++(int) {
		(* static_cast<Iterator<Vp>*>(this))++;
		return *this;
	}

	unsigned int count() const {
		return static_cast<const Iterator<Vp>*>(this)->count();
	}

	operator Vp () {
		return *(* static_cast<Iterator<Vp>*>(this));
	}

};

}

#endif
