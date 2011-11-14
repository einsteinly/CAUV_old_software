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

#ifndef __CAUV_F_MANAGED_ELEMENT_H__
#define __CAUV_F_MANAGED_ELEMENT_H__

namespace cauv{
namespace gui{
namespace f{

class Manager;

// GUI element managed by (pipeline gui) Manager. Subclassing this ensures that
// a handle to the manager is available, e.g. for signal/slot connections
//
class ManagedElement{
    public:
        ManagedElement(ManagedElement const& other);
        ManagedElement(Manager& manager);    

    protected:
        Manager& manager();

    private:
        Manager& m_manager;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_MANAGED_ELEMENT_H__


