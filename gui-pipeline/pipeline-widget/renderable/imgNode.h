#ifndef __IMGNODE_RENDERABLE_H__
#define __IMGNODE_RENDERABLE_H__

#include "node.h"

#include <common/image.h>

namespace pw{

class TexImg;

class Img: public Renderable{
    public:
        Img(container_ptr_t c);
        virtual void draw(bool);
        virtual BBox bbox();
        void display(Image const& img);

    private:
        BBox m_bbox;
        boost::shared_ptr<TexImg> m_img;
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

#endif // ndef __IMGNODE_RENDERABLE_H__
