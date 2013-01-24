/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GEMINI_NODE_H__
#define __CAUV_GEMINI_NODE_H__


#include <common/cauv_node.h>

namespace cauv{

namespace po = boost::program_options;

class GeminiSonar;

class GeminiNode: public CauvNode{
    public:
        GeminiNode();
    protected:
        virtual void onRun(); 
        virtual void addOptions(po::options_description& desc,
                                po::positional_options_description& pos);
        virtual int useOptionsMap(po::variables_map& vm,
                                  po::options_description& desc);
    private:
        boost::shared_ptr<GeminiSonar> m_sonar;
};

} // namespace cauv

#endif // ndef __CAUV_GEMINI_NODE_H__
