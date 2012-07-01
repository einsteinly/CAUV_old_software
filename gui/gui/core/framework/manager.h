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

#ifndef __CAUV_GUINODEMANAGER_H__
#define __CAUV_GUINODEMANAGER_H__

#include <map>
#include <stdexcept>

#include <boost/shared_ptr.hpp>

#include <debug/cauv_debug.h>
#include <liquid/node.h>

namespace cauv {
namespace gui {

class Node;

template <class T>
class Manager{
public:
    Manager<T>(boost::shared_ptr<Node> node, T* item){
        m_mapping[node] = item;
    }

    static T * liquidNode(boost::shared_ptr<Node> node){
        if(m_mapping.find(node) != m_mapping.end())
            return m_mapping[node];
        else return NULL;
    }

    typedef std::map<boost::shared_ptr<Node>, T*> t_map;

    void unregister(boost::shared_ptr<Node> const& node){
        if(m_mapping.find(node) != m_mapping.end())
            unregister(m_mapping[node]);
    }

    void unregister(T* ln){
        typename Manager<T>::t_map::iterator iter;
        for (iter = m_mapping.begin(); iter != m_mapping.end(); ++iter) {
            if(iter->second == ln) {
                m_mapping.erase(iter);
            }
        }
    }

protected:
    static t_map m_mapping;
};

template<class T>
std::map<boost::shared_ptr<Node>, T*> Manager<T>::m_mapping;

} // namespace gui
} // namespace cauv


#endif // __CAUV_GUINODEMANAGER_H__
