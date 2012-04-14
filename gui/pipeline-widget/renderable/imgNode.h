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

#ifndef __IMGNODE_RENDERABLE_H__
#define __IMGNODE_RENDERABLE_H__

#include "node.h"
#include "resizeable.h"

#include <boost/thread/mutex.hpp>

#include <common/msg_classes/image.h>

namespace cauv{
namespace pw{

class TexImg;

class Img: public Resizeable{
    public:
        Img(container_ptr_t c);
        virtual void draw(drawtype_e::e flags);
        void display(Image const& img);

    private:
        boost::shared_ptr<TexImg> m_img, m_next_img;
        boost::mutex m_img_mutex;
};

class ImgNode: public Node{
    public:
        ImgNode(container_ptr_t c, pw_ptr_t pw, boost::shared_ptr<NodeAddedMessage const> m);
        ImgNode(container_ptr_t c, pw_ptr_t pw, node_id const& id, NodeType::e const& nt);
        void display(Image const& img);
    
    private:
        boost::shared_ptr<Img> m_img;
};

} // namespace pw
} // namespace cauv

#endif // ndef __IMGNODE_RENDERABLE_H__
