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

#include "imgNode.h"

#include <QtOpenGL>
#include <GL/glu.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>

#include <common/image.h>
#include <debug/cauv_debug.h>

#include "../util.h"

namespace cauv{
namespace pw{

// nasty solution, but TexImg objects might be destroyed in threads other than
// the gui thread
static struct _TFD: public std::vector<GLuint>{
    ~_TFD(){
        deleteAndClear();
    }
    void deleteAndClear(){
        if(int n = size()){
            glDeleteTextures(n, &operator[](0));
            glCheckError();
            clear();
        }
    }
} Textures_For_Deleting;

// TODO: something more sophisticated; tiling for large images, dynamic
// re-allocation of texture ids, checking the capabilities of the graphics
// system, rendering at arbitrary resolution, etc, etc
class TexImg{
    public:
        TexImg(Image const& img)
            : m_img(boost::make_shared<Image>(img)), m_tex_id(0){
            int chs = img.mat().channels();
            if(img.mat().depth() != CV_8U || (chs!=3 && chs!=4 && chs!=1) ||
               !img.mat().isContinuous() || !img.mat().rows || !img.mat().cols){
                error() << "incompatible image type" << img;
                m_img.reset();
            }
        }

        ~TexImg(){
            //TODO: does this need any locking??!
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
            int err = 0;
            boost::shared_ptr<Image> img = m_img;
            if(!img){
                error() << __func__ << __LINE__ << "no image";
                return;
            }
            m_img.reset();
            
            debug(4) << "Generating teximg texture";

            glGenTextures(1, &m_tex_id);
            glBindTexture(GL_TEXTURE_2D, m_tex_id);

            glCheckError();

            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);

            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
          
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1); 
            
            GLint max_size = 0;
            glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max_size);
            
            glCheckError();

            debug(5) << "max texture size:" << max_size;
            if(!max_size){
                error() << "cannot create texture";
                return;
            }
            
            cv::Mat t = img->mat(), m;
            int w = t.cols;
            int h = t.rows;
            if (h <= max_size && w <= max_size)
            {
                m = t;
            }
            else
            {
                debug() << "original img size:" << w << h;
                int w_resized, h_resized;
                if(h > w){
                    h_resized = max_size;
                    w_resized = h_resized * w / h;
                }else{
                    w_resized = max_size;
                    h_resized = w_resized * h / w;
                }
                w = w_resized;
                h = h_resized;
                debug() << "resized:" << w << h;
                cv::resize(t, m, cv::Size(w,h), 0, 0, cv::INTER_LANCZOS4);
            }

            GLenum tex_type = GL_UNSIGNED_BYTE;
            switch(m.type() & CV_MAT_DEPTH_MASK){
                case CV_8S:
                    warning() << "_genTexture(): signed byte image type not supported: using unsigned";
                    // case fall through
                case CV_8U:
                    tex_type = GL_UNSIGNED_BYTE;
                    break;
                case CV_16S:
                    tex_type = GL_SHORT;
                    break;
                case CV_16U:
                    tex_type = GL_UNSIGNED_SHORT;
                    break;
                case CV_32S:
                    tex_type = GL_INT;
                    break;
                case CV_32F:
                    tex_type = GL_FLOAT;
                    break;
                case CV_64F:
                    warning() << "_genTexture(): double image type not supported: trying anyway...";
                    tex_type = GL_DOUBLE;
                    break;
                default:
                    error() << "unknown OpenCV image type depth:";
            }
            
            if(m.channels() == 4)
                err = gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_BGRA, tex_type, m.data);
            else if(m.channels() == 3)
                err = gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_BGR, tex_type, m.data);
            else if(m.channels() == 1)
                err = gluBuild2DMipmaps(GL_TEXTURE_2D, 3, w, h, GL_LUMINANCE, tex_type, m.data);        
            if(err){
                error() << "GLU error:" << err;
            }
            
            glCheckError();
        }

        boost::shared_ptr<Image> m_img;
        GLuint m_tex_id;
};

} // namespace pw
} // namespace cauv

using namespace cauv::pw;

Img::Img(container_ptr_t c)
    : Resizeable(c, BBox(0, 0, 300, 200), BBox(10, 10), BBox(1200, 800)){
}

void Img::draw(drawtype_e::e flags){

    // delete any textures waiting to be deleted
    Textures_For_Deleting.deleteAndClear();
    
    glColor(Colour(0.6, 0.05, 0.1, 0.5));
    glBox(m_bbox);
    
    CAUV_LOCK(m_img_mutex) {
        if (m_next_img)
        {
            m_img.swap(m_next_img);
            m_next_img.reset();
        }
    }

    if(!(flags & drawtype_e::picking) && m_img)
        m_img->draw(m_bbox);
    Resizeable::drawHandle();
}

void Img::display(Image const& img){
    debug(6) << __func__ << "enter";
    boost::shared_ptr<TexImg> next_img = boost::make_shared<TexImg>(img);
    
    CAUV_LOCK(m_img_mutex) {
        m_next_img = next_img;
    }

    aspect(double(img.mat().cols) / img.mat().rows);
    m_context->postRedraw(0);
    debug(6) << __func__ << "exit";
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
    m_img->display(img);
}

