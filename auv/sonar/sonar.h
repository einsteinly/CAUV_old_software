#ifndef __CAUV_SONAR_H__
#define __CAUV_SONAR_H__

#include <string>

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

#include "seanet_sonar.h"

namespace cauv{

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

