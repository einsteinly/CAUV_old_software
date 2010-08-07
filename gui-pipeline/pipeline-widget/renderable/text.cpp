#include "text.h"

#include "FTGL/ftgl.h"

#include <QtOpenGL>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <debug/cauv_debug.h>

using namespace pw;

Text::Text(container_ptr_t c, std::string const& text, std::string const& font, int pt)
    : Renderable(c), std::string(text), m_bbox(),
      m_font(std::make_pair(font, pt)), m_colour(Colour(0)){
}
        
void Text::draw(drawtype_e::e){
    if(!font()) return;

    glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT |
                 GL_DEPTH_BUFFER_BIT | GL_ENABLE_BIT);
    glPushMatrix();
    
    glEnable(GL_TEXTURE_2D);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor(m_colour);
    font()->Render(c_str());
    glPrintErr();
    
    glPopMatrix();
    glPopAttrib();
}

BBox Text::bbox(){
    if(bboxFont()){
        if(!m_bbox)
            updateBbox();
        return BBox(m_bbox->Lower().X(), m_bbox->Lower().Y(),
                    m_bbox->Upper().X(), m_bbox->Upper().Y());
    }else{
        return BBox(0, 0, 10, 1); 
    }
}

void Text::updateBbox(){
    m_bbox = boost::make_shared<FTBBox>(bboxFont()->BBox(c_str()));
}

void Text::colour(Colour const& c){
    m_colour = c;
}

typedef boost::shared_ptr<FTFont> font_ptr;
typedef std::pair<std::string, int> face_pt_pair_t;

// TODO: find a home for this
#include <set>
bool onceCheck(const char* name){
    // Note that we compare names by POINTER: names should be constant string
    // literals, and in order to use the same name from multiple places
    // constant compression must be on.
    static std::set<const char*> once;
    if(once.count(name))
        return false;
    once.insert(name);
    return true;
}
#define once(name) if(onceCheck(#name))

template<typename font_T>
static boost::shared_ptr<FTFont> font(face_pt_pair_t const& id){
    static std::map<face_pt_pair_t, font_ptr> fonts;
    std::map<face_pt_pair_t, font_ptr>::const_iterator i = fonts.find(id);
    if(i != fonts.end()){
        return i->second;
    }
    font_ptr new_f = boost::make_shared<font_T>(id.first.c_str());
    if(new_f->Error()){
        once(unable_to_open_font)
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

boost::shared_ptr<FTFont> Text::font(face_pt_pair_t const& id){
    return ::font<FTTextureFont>(id);
}


boost::shared_ptr<FTFont> Text::bboxFont(face_pt_pair_t const& id){
    return ::font<FTPixmapFont>(id);
}




