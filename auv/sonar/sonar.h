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

#ifndef __CAUV_SONAR_H__
#define __CAUV_SONAR_H__

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

namespace cauv{

class SeanetSonar;

class SonarNode : public CauvNode
{
	public:
		SonarNode();
	
    protected:
        boost::shared_ptr<SeanetSonar> m_sonar;
        
        virtual void onRun();
        virtual void addOptions(boost::program_options::options_description& desc,
                                boost::program_options::positional_options_description& pos);
        virtual int useOptionsMap(boost::program_options::variables_map& vm,
                                  boost::program_options::options_description& desc);
        
};

} // namespace cauv

#endif //ndef __CAUV_SONAR_H__

