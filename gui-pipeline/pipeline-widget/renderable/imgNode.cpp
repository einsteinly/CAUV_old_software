#include "imgNode.h"

#include <QtOpenGL>

#include <common/image.h>


namespace pw{

// nasty solution, but TexImg objects might be destroyed in threads other than
// the gui thread
static std::vector<GLuint> Textures_For_Deleting;

// TODO: something more sophisticated; tiling for large images, dynamic
// re-allocation of texture ids, checking the capabilities of the graphics
// system, rendering at arbitrary resolution, etc, etc
class TexImg{
    public:
        TexImg(Image const& img)
            : m_img(boost::make_shared<Image>(img)), m_tex_id(0){
            int chs = img.cvMat().channels();
            if(img.cvMat().depth() != CV_8U || (chs!=3 && chs!=4 && chs!=1) ||
               !img.cvMat().isContinuous()){
                error() << "incompatible image type" << img;
                m_img.reset();
            }
        }

        ~TexImg(){
            if(m_tex_id)
                Textures_For_Deleting.push_back(m_tex_id);
        }
        
        void draw(BBox const& b){
            if(!m_tex_id && m_img)
                _genTexture();

            if(m_tex_id){
                glEnable(GL_TEXTURE_2D);
                glBindTexture(GL_TEXTURE_2D, m_tex_id);
                glColor(Colour(1));
                glBegin(GL_QUADS);
                glTexCoord2d(0.0,1.0); glVertex2d(b.min.x, b.min.y);
                glTexCoord2d(1.0,1.0); glVertex2d(b.max.x, b.min.y);
                glTexCoord2d(1.0,0.0); glVertex2d(b.max.x, b.max.y);
                glTexCoord2d(0.0,0.0); glVertex2d(b.min.x, b.max.y);
                glEnd();
                glDisable(GL_TEXTURE_2D);
            }else{
                glColor(Colour(1, 0, 0, 0.5));
                glBox(b);
            }
        }

    private:
        void _genTexture(){
            if(!m_img){
                error() << __func__ << __LINE__ << "no image";
                return;
            }
            glGenTextures(1, &m_tex_id);
            glBindTexture(GL_TEXTURE_2D, m_tex_id);

            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
           
            cv::Mat m;
            const int w = 400;
            const int h = w * m_img->cvMat().rows / m_img->cvMat().cols;
            cv::resize(m_img->cvMat(), m, cv::Size(w, h), 0, 0, cv::INTER_LANCZOS4);
            
            if(m.channels() == 4)
                gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_BGRA, GL_UNSIGNED_BYTE, m.data);
            else if(m.channels() == 3)
                gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_BGR, GL_UNSIGNED_BYTE, m.data);
            else if(m.channels() == 1)
                gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE, m.data);

            m_img.reset();
        }

        boost::shared_ptr<Image> m_img;
        GLuint m_tex_id;
};

} // namespace pw

using namespace pw;

Img::Img(container_ptr_t c)
    : Renderable(c), m_bbox(0, 0, 300, 200){
}

void Img::draw(bool picking){
    if(int n = Textures_For_Deleting.size()){
        glDeleteTextures(n, &Textures_For_Deleting[0]);
        Textures_For_Deleting.clear();
    }
    glColor(Colour(0.2, 0.5));
    glBox(m_bbox);
    if(!picking && m_img)
        m_img->draw(m_bbox);
}

BBox Img::bbox(){
    return m_bbox;
}

void Img::display(Image const& img){
    m_img = boost::make_shared<TexImg>(img);
    m_context->postRedraw();
}


ImgNode::ImgNode(container_ptr_t c, pw_ptr_t pw, boost::shared_ptr<NodeAddedMessage const> m)
    : Node(c, pw, m), m_img(boost::make_shared<Img>(this)){
    m_contents.push_back(m_img);
    m_extra_stuff.insert(m_img);
    refreshLayout();
}

ImgNode::ImgNode(container_ptr_t c, pw_ptr_t pw, node_id const& id, NodeType::e const& nt)
    : Node(c, pw, id, nt), m_img(boost::make_shared<Img>(this)){
    m_contents.push_back(m_img);
    m_extra_stuff.insert(m_img);
    refreshLayout();
}

void ImgNode::display(Image const& img){
    debug() << "ImgNode::display" << img;
    m_img->display(img);
}

