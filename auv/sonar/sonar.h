#ifndef __SONAR_H__
#define __SONAR_H__

#include <string>

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

#include "seanet_sonar.h"

class SonarNode : public CauvNode
{
	public:
		SonarNode(const std::string& device);
	
    protected:
        boost::shared_ptr<SeanetSonar> m_sonar;
        
        virtual void onRun();
};

#endif

