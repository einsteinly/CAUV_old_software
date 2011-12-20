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

#ifndef __CAUV_BIMAP_WHICH_IS_BETTER_THAN_BOOST_BIMAP_H__
#define __CAUV_BIMAP_WHICH_IS_BETTER_THAN_BOOST_BIMAP_H__

#include <utility>
#include <stdexcept>

#include <boost/multi_index_container.hpp> 
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>

namespace cauv{

namespace bmi = boost::multi_index;

template<typename L, typename R>
class bimap{
    public: 
        struct value_type{
            value_type(L const& l, R const& r) : left(l), right(r){ }
            L left;
            R right;
        };
        struct Left{};
        struct Right{};
        typedef bmi::multi_index_container<
            value_type,
            bmi::indexed_by<
                bmi::ordered_unique<
                    bmi::tag<Left>, bmi::member<value_type,L,&value_type::left >
                >,
                bmi::ordered_unique<
                    bmi::tag<Right>, bmi::member<value_type,R,&value_type::right>
                >
            >
        > mmap_type;
        typedef typename mmap_type::template index<Right>::type::iterator right_iterator;
        typedef typename mmap_type::template index<Right>::type::const_iterator right_const_iterator;
        typedef typename mmap_type::template index<Left>::type::iterator left_iterator;
        typedef typename mmap_type::template index<Left>::type::const_iterator left_const_iterator;

        // <3 Clang

        const typename mmap_type::template index<Left>::type& left() const{ return m_map.template get<Left>(); }
        typename mmap_type::template index<Left>::type& left(){ return m_map.template get<Left>(); }

        const typename mmap_type::template index<Right>::type& right() const{ return m_map.template get<Right>(); }
        typename mmap_type::template index<Right>::type& right(){ return m_map.template get<Right>(); }
        
        const R& operator[](L const& key) const {
            left_const_iterator i = left().find(key);
            if(i == left().end())
                throw std::runtime_error("cauv::bimap::operator[] does not create values");
            return i->right;
        }
        const L& operator[](R const& key) const {
            right_const_iterator i = right().find(key);
            if(i == right().end())
                throw std::runtime_error("cauv::bimap::operator[] does not create values");
            return i->left;
        }
 
        right_const_iterator find(R const& key) const{ return right().find(key); }
        left_const_iterator find(L const& key) const{ return left().find(key); }

        right_iterator find(R const& key) { return right().find(key); }
        left_iterator find(L const& key) { return left().find(key); }
        
        void insert(value_type const& v){ m_map.insert(v); }

        void erase(R const& key){ m_map.template get<Left>().erase(key); }
        void erase(L const& key){ m_map.template get<Right>().erase(key); }

        void clear(){ m_map.clear(); }

        std::size_t size() const{ return m_map.size(); }

        
    private:

        mmap_type m_map;
};

} // namespace cauv

#endif // ndef __CAUV_BIMAP_WHICH_IS_BETTER_THAN_BOOST_BIMAP_H__
