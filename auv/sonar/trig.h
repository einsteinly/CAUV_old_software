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


#ifndef __CAUV_SONAR_TRIG_H__
#define __CAUV_SONAR_TRIG_H__

#include <vector>
#include <cmath>

namespace cauv{

// This class is const, so there's no point in having more than the one
// instance:
const static class SeanetCachedTrig{
    public:
        SeanetCachedTrig()
            : m_sins(precalculate_sins()){
        }

        static std::vector<float> precalculate_sins(){
            std::vector<float> ret;
            ret.reserve(1600);
            for (int b = 0; b < 1600; ++b)
                ret.push_back(std::sin((M_PI / 3200) * b));
            return ret;
        }

        float sin(int bearing) const{
            bearing = bearing % 6400;
            if (bearing < 0)
                bearing += 6400;

            if (bearing < 1600)
                return m_sins[bearing];
            else if (bearing < 3200)
                return m_sins[3200 - bearing];
            else if (bearing < 4800)
                return -m_sins[bearing - 3200];
            else
                return -m_sins[6400 - bearing];
        }

        float cos(int bearing) const{
            return sin(bearing + 1600);
        }

    private:
        const std::vector<float> m_sins;
} seanet_cached;

class GemCachedTrig{
    public:
        GemCachedTrig()
            : sins(), coss(), sins_start_from(0){
        }

        static float sin_nocache(int32_t bearing){
            return -std::sin(M_PI * bearing / (3200.0*0x10000));
        }
        static float cos_nocache(int32_t bearing){
            return -sin_nocache(bearing + 1600*0x10000);
        }
        float sin_idx(uint32_t bearing_idx) const{ return sins[bearing_idx]; }
        float cos_idx(uint32_t bearing_idx) const{ return coss[bearing_idx]; }

        void ensureTablesFor(std::vector<int32_t> bearings){
            if(bearings[0] == sins_start_from &&
               bearings.size() == sins.size()){
                return;
            }
            sins.clear();
            coss.clear();
            sins.reserve(bearings.size());
            coss.reserve(bearings.size());
            for(uint32_t i = 0; i < bearings.size(); i++){
                sins[i] = sin_nocache(bearings[i]);
                coss[i] = cos_nocache(bearings[i]);
            }
        }

    private:
        std::vector<float> sins;
        std::vector<float> coss;
        int32_t sins_start_from;
};

class CachedTrig{
    public:
        CachedTrig()
            : sins(), coss(), sins_start_from(0){
        }

        float sin_idx(uint32_t angle_idx) const{ return sins[angle_idx]; }
        float cos_idx(uint32_t angle_idx) const{ return coss[angle_idx]; }

        void ensureTablesFor(std::vector<float> angles){
            if(angles[0] == sins_start_from &&
               angles.size() == sins.size()){
                return;
            }
            sins.clear();
            coss.clear();
            sins.reserve(angles.size());
            coss.reserve(angles.size());
            for(uint32_t i = 0; i < angles.size(); i++){
                sins[i] = std::sin(angles[i]);
                coss[i] = std::cos(angles[i]);
            }
        }

    private:
        std::vector<float> sins;
        std::vector<float> coss;
        float sins_start_from;
};

/*
static float gem_sin_safe(int32_t bearing){
    // !!! TODO: better caching scheme....
    static std::map<int32_t, float> sin_cache;

    std::map<int32_t, float>::const_iterator i = sin_cache.find(bearing);
    if(i == sin_cache.end()){
        const float r = gem_sin_nocache(bearing);
        sin_cache[bearing] = r;
        if(sin_cache.size() > 100000){
            std::map<int32_t, float>::iterator to_remove = sin_cache.lower_bound(rand() % 6400*0x10000);
            if(to_remove == sin_cache.end()){
                // !!! really not good: values probably lie outside 0--6400*0x10000
                sin_cache.erase(sin_cache.begin());
            }else{
                sin_cache.erase(to_remove);
            }
        }
        return r;
    }else{
        return i->second;
    }
}

static float gem_cos_safe(int32_t bearing){
    return gem_sin_safe(bearing + 1600*0x10000);
}
*/

} // namespace cauv

#endif // ndef __CAUV_SONAR_TRIG_H__

