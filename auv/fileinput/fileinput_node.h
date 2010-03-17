#ifndef __FILEINPUT_NODE_H__
#define __FILEINPUT_NODE_H__

#include <string>

#include <boost/shared_ptr.hpp>

#include <common/cauv_node.h>

class FileinputNode : public CauvNode
{
    public:
        FileinputNode(std::string const& fname);

    protected:
		void onRun();

    private:
        std::string m_fname;
};

#endif//__FILEINPUT_NODE_H__
