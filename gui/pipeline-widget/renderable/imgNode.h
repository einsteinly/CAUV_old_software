/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __IMGNODE_RENDERABLE_H__
#define __IMGNODE_RENDERABLE_H__

#include "node.h"
#include "resizeable.h"

#include <boost/thread/mutex.hpp>

#include <common/msg_classes/image.h>

namespace cauv{
namespace gui{
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
} // namespace gui
} // namespace cauv

#endif // ndef __IMGNODE_RENDERABLE_H__
