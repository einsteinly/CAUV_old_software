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

#include <limits>

#include <utility/serialisation.h>
#include <utility/serialisation-types.h>
#include <utility/streamops/vector.h>
#include <utility/streamops/map.h>
#include <debug/cauv_debug.h>

#include <boost/make_shared.hpp>

using namespace cauv;


int main(){
    svec_ptr sv = boost::make_shared<svec_t>();
    

    /***  integral types   ***/
    uint8_t  a = 0xfa;
    uint32_t b = 0xfeedf00d;
    uint16_t c = 0xbeef;
    int8_t   d = 0x12;
    int32_t  e = 0x34567890;
    int16_t  f = 0x7bcd;

    serialise(sv, a);
    serialise(sv, b);
    serialise(sv, c);
    serialise(sv, d);
    serialise(sv, e);
    serialise(sv, f);

    a = b = c = d = e = f = 0;

    deserialise(sv, 0x0, a);
    deserialise(sv, 0x1, b);
    deserialise(sv, 0x5, c);
    deserialise(sv, 0x7, d);
    deserialise(sv, 0x8, e);
    deserialise(sv, 0xc, f);

    if (a != 0xfa)
        error() << "uint8 failed:" << a << "!=" << 0xfa;
    if (b != 0xfeedf00d)
        error() << "uint32 failed:" << b << "!=" << 0xfeedf00d;
    if (c != 0xbeef)
        error() << "uint16 failed:" << c << "!=" << 0xbeef;
    if (d != 0x12)
        error() << "int8 failed:" << d << "!=" << 0x12;
    if (e != 0x34567890)
        error() << "int32 failed:" << e << "!=" << 0x34567890;
    if (f != 0x7bcd)
        error() << "int16 failed:" << f << "!=" << 0x7bcd;

    sv->clear(); 
    /***  floating point types   ***/
    float  af = 0.1f;
    float  bf = 0.123f;
    double cf = -1.245e78;
    float  df = -std::numeric_limits<float>::infinity(); // inf should be serialisable
    double ef = std::numeric_limits<double>::infinity();
    //double ff = 0.0/0.0; // NaN is not required to be serialisable but probably is)
    //float  gf = 0.0f/0.0f;

    serialise(sv, af);
    serialise(sv, bf);
    serialise(sv, cf);
    serialise(sv, df);
    serialise(sv, ef);
    //serialise(sv, ff);
    //serialise(sv, gf);

    af = bf = cf = df = ef = /*ff = gf =*/ 0;

    deserialise(sv, 0x0, af);
    deserialise(sv, 0x4, bf);
    deserialise(sv, 0x8, cf);
    deserialise(sv, 0x10, df);
    deserialise(sv, 0x14, ef);
    //deserialise(sv, 0x1c, ff);
    //deserialise(sv, 0x24, gf);

    if (af != 0.1f)
        error() << "float failed:" << af << "!=" << 0.1f;
    if (bf != 0.123f)
        error() << "float failed:" << bf << "!=" << 0.123f;
    if (cf != -1.245e78)
        error() << "double failed:" << cf << "!=" << -1.245e78;
    if (df != -std::numeric_limits<float>::infinity())
        error() << "float failed:" << df << "!=" << -std::numeric_limits<float>::infinity();
    if (ef != std::numeric_limits<double>::infinity())
        error() << "double failed:" << ef << "!=" << std::numeric_limits<double>::infinity();
    //if (ff != 0.0/0.0)
    //    error() << "double failed:" << ff << "!=" << 0.0/0.0;
    //if (gf != 0.0f/0.0f)
    //    error() << "float failed:" << gf << "!=" << 0.0f/0.0f;

    sv->clear();

    /***  vectors   ***/
    std::vector<int> va;
    va.push_back(12);
    va.push_back(12345);
    va.push_back(0xfeed);
    va.push_back(-1234567);

    serialise(sv, va);

    std::vector<int> vb;

    deserialise(sv, 0, vb);
    
    if(vb != va)
        error() << "vec<int> failed:" << vb << "!=" << va;
    
    sv->clear();
    /* misaligned vector */
    serialise(sv, uint8_t('a'));
    serialise(sv, va);

    uint8_t mv_a;
    vb.clear();

    deserialise(sv, 1, vb);
    deserialise(sv, 0, mv_a);
    
    if(mv_a != 'a')
        error() << "misaligned vec<int> overwrote data" << mv_a << "!=" << 'a';
    if(vb != va)
        error() << "misaligned vec<int> failed:" << vb << "!=" << va;

    sv->clear();    
    /* vector of vector */
    std::vector<std::vector<uint8_t> > vc;
    vc.push_back(std::vector<uint8_t>(3, 'a'));
    vc.push_back(std::vector<uint8_t>(1, 'b'));
    vc.push_back(std::vector<uint8_t>(0, 'c'));
    vc.push_back(std::vector<uint8_t>(12, 'd'));
    vc.push_back(std::vector<uint8_t>(4, 'e'));
    vc.push_back(std::vector<uint8_t>(4, 'f'));
    vc.push_back(std::vector<uint8_t>(1, 'g'));
    vc.push_back(std::vector<uint8_t>(3, 'h'));

    serialise(sv, vc);

    std::vector<std::vector<uint8_t> > vd;

    deserialise(sv, 0, vd);

    if(vd != vc)
        error() << "vec<vec<uint8_t>> failed:" << vc << "!=" << vd;
    debug() << vc;

    sv->clear();
    /***  maps  ***/
    std::map< int, std::vector<char> > ma;
    ma[12837612] = std::vector<char>(3, 'A');
    ma[-123] = std::vector<char>(4, 'B');
    ma[3] = std::vector<char>(2, 'q');
    ma[4] = std::vector<char>(0, 'w');
    ma[5] = std::vector<char>(3, 'e');
    ma[6] = std::vector<char>(1, 'r');
    ma[0] = std::vector<char>(12, 't');

    serialise(sv, ma);

    std::map< int, std::vector<char> > mb;

    deserialise(sv, 0, mb);

    if(mb != ma)
        error() << "map<int,vec<uint>> failed:" << mb << "!=" << ma;
    debug() << mb;

}

