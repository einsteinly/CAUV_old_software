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

#include "managedElement.h"

using namespace cauv;
using namespace cauv::gui;

ManagedElement::ManagedElement(ManagedElement const& other)
    : m_manager(other.m_manager){
}

ManagedElement::ManagedElement(Manager& manager)
    : m_manager(manager){
}

Manager& ManagedElement::manager(){
    return m_manager;
}
        
