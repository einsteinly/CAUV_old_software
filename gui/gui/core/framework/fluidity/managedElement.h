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


