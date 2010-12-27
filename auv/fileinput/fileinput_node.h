#ifndef __CAUV_FILEINPUT_CAUVNODE_H__
#define __CAUV_FILEINPUT_CAUVNODE_H__

#include <string>

#include <common/cauv_node.h>
#include <common/image.h>

namespace cauv{

class FileinputCauvNode : public CauvNode
{
    public:
        FileinputCauvNode(std::string const& fname);

    protected:
		void onRun();

    private:
        std::string m_fname;
        Image m_img;
};

} // namsepace cauv

#endif // __CAUV_FILEINPUT_CAUVNODE_H__
