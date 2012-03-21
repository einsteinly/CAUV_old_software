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

#ifndef __CAUV_RANDOM_H__
#define __CAUV_RANDOM_H__

#include <vector>
#include <set>
#include <stdexcept>

namespace cauv {

    int random(int min, int max);
    int random(int min, int max, unsigned int seed);

    template<typename T>
    std::vector<T> randomSubset(const std::vector<T>& src, typename std::vector<T>::size_type size)
    {
        if (size > src.size())
            throw std::range_error("Subset size out of range");

        std::vector<T> ret;
        std::set<int> vals;

        for (size_t j = src.size() - size; j < src.size(); ++j)
        {
            int rand = random(0, j); // generate a random integer in range [0, j]

            if (vals.find(rand) == vals.end())
                ret.push_back(src[rand]);
            else
                ret.push_back(src[j]);
        }

        return ret;
    }

    template<typename T>
    std::vector<T> randomSubset(const std::vector<T>& src, typename std::vector<T>::size_type size, unsigned int seed)
    {
        if (size > src.size())
            throw std::range_error("Subset size out of range");

        std::vector<T> ret;
        std::set<int> vals;

        for (size_t j = src.size() - size; j < src.size(); ++j)
        {
            int rand = random(0, j, seed+j); // generate a random integer in range [0, j]

            if (vals.find(rand) == vals.end())
                ret.push_back(src[rand]);
            else
                ret.push_back(src[j]);
        }

        return ret;
    }

}

#endif//__CAUV_RANDOM_H__
