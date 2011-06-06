#ifndef __CAUV_TESTABLE_H__
#define __CAUV_TESTABLE_H__


/* Synopsis:
 * Make it easy to provide a safe conversion to bool, without requiring virtual
 * functions.
 * 
 * Usage:
 * class Derived: public TestableBase<Derived>{
 *     public:
 *         Derived()
 *             : TestableBase<Derived>(*this){
 *         }
 *         bool valid() const { .... }
 * };
 * 
 */

// TODO: since copy construct requires static cast anyway, might as well do
// away with m_ref, and use a static cast whenever we want to call valid()
template<typename Derived_T_With_valid_func>
class TestableBase{
        typedef void (TestableBase::*bool_t)() const;
    public:
        operator bool_t() const {
            return static_cast<Derived_T_With_valid_func const*>(this)->valid()?
                &TestableBase::this_type_does_not_support_comparisons :
                NULL;
        }
    protected:
        ~TestableBase(){ }
    private:
        void this_type_does_not_support_comparisons() const {}
};

/* old version (also works, but uses reference)
template<typename Derived_T_With_valid_func>
class TestableBase{
        typedef void (TestableBase::*bool_t)() const;
    public:
        explicit TestableBase(Derived_T_With_valid_func const& d_ref)
            : m_ref(d_ref){
        }

        TestableBase(TestableBase const&)
            : m_ref(static_cast<Derived_T_With_valid_func const&>(*this)){
        }

        TestableBase& operator=(TestableBase const& other){
            if (this != &other){
                this->TestableBase::~TestableBase(); // destroy base only! 
                new (this) TestableBase(other); // placement new
            }
            return *this;
        }

        operator bool_t() const {
            return m_ref.valid()? &TestableBase::this_type_does_not_support_comparisons : NULL;
        }
    private:

        void this_type_does_not_support_comparisons() const {}
        Derived_T_With_valid_func const& m_ref;
};
*/

template <typename dT, typename T> 
bool operator!=(TestableBase<dT> const& l, T const&){
    l.comparison_not_allowed();	
    return false;	
} 
template <typename dT, typename T>
bool operator==(TestableBase<dT> const& l, T const&){
    l.comparison_not_allowed();
    return false;		
}


#endif  // ndef __CAUV_TESTABLE_H__
