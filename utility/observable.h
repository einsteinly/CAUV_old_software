/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __OBSERVABLE_H__
#define __OBSERVABLE_H__

#include <list>

#include <boost/shared_ptr.hpp>

template<typename T>
class Observable
{
    public:
        typedef boost::shared_ptr<T> observer_ptr_t;

        void addObserver(observer_ptr_t o)
        {
            m_observers.push_back(o);
        }
        void removeObserver(observer_ptr_t o)
        {
            m_observers.remove(o);
        }
        void clearObservers()
        {
            m_observers.clear();
        }

    protected:
        std::list<observer_ptr_t> m_observers;
};

#endif//__OBSERVABLE_H__
