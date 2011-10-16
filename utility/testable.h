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
