#include "text.h"

#include "ftgl/ftgl.h"

#include <QtOpenGL>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <common/debug.h>

Text::Text(PipelineWidget& p, std::string const& text, std::string const& font, int pt)
    : Renderable(boost::ref(p)), m_bbox(), m_font(std::make_pair(font, pt)), m_text(text){
}
        
void Text::draw(){
    if(!font()) return;

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glColor4f(0.0, 0.0, 0.0, 1.0);
    
    glPushMatrix();
    font()->Render(m_text.c_str());
    glPopMatrix();

    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
}

BBox Text::bbox(){
    if(font()){
        if(!m_bbox){
            m_bbox = boost::make_shared<FTBBox>(font()->BBox(m_text.c_str()));
        }
        BBox r = {m_bbox->Lower().X(), m_bbox->Lower().Y(),
                  m_bbox->Upper().X(), m_bbox->Upper().Y()};
        return r;
    }else{
        BBox r = {0, 0, 10, 1};
        return r; 
    }
}

boost::shared_ptr<FTFont> Text::font(face_pt_pair_t const& id){
    static std::map<face_pt_pair_t, font_ptr> fonts;
    std::map<face_pt_pair_t, font_ptr>::const_iterator i = fonts.find(id);
    if(i != fonts.end()){
        return i->second;
    }
    font_ptr new_f = boost::make_shared<FTTextureFont>(id.first.c_str());
    if(new_f->Error()){
        error() << "Unable to open font file:" << id.first.c_str();
        new_f.reset();
        // TODO: fallback font?
    }else if (!new_f->FaceSize(id.second)){
        error() << "Unable to set size=" << id.second << "on font" << id.first;
        new_f.reset();
    }else{
        fonts[id] = new_f;
    }
    return new_f;
}



