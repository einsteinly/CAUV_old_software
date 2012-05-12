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

#ifndef __CAUV_FILEINPUT_CAUVNODE_H__
#define __CAUV_FILEINPUT_CAUVNODE_H__

#include <string>

#include <common/cauv_node.h>
#include <common/msg_classes/image.h>

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
