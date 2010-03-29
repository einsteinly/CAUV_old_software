#ifndef __TEXT_RENDERABLE_H__
#define __TEXT_RENDERABLE_H__

#include "../renderable.h"

#include <utility>
#include <string>

class FTFont;
class FTBBox;

class Text: public Renderable, public std::string{
    public:
        Text(container_ptr_t c, std::string const& text,
             std::string const& font="LiberationMono-Regular.ttf", int pt=12);
        
        virtual bool acceptsMouseEvents(){ return false; }
        virtual void draw(bool);
        virtual BBox bbox();
        virtual void updateBbox();
    
    protected:
        typedef boost::shared_ptr<FTFont> font_ptr;
        typedef std::pair<std::string, int> face_pt_pair_t;
        static font_ptr font(face_pt_pair_t const& id);

        // FIXME: FTGL can call glTex... functions when calculating bbox sizes
        // using a FTTextureFont (or, infact, FTBufferFont) (it initialises a
        // texture for each glyph), that causes all sorts of badness if the
        // current thread isn't the gui thread, so use a different sort of font
        // to determine bboxes:
        static font_ptr bboxFont(face_pt_pair_t const& id);


        font_ptr font(){ return font(m_font); }
        font_ptr bboxFont(){ return bboxFont(m_font); }

    private:
        boost::shared_ptr<FTBBox> m_bbox;
        face_pt_pair_t m_font;
};

#endif // ndef __TEXT_RENDERABLE_H__

