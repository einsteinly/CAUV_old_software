#ifndef __TEXT_RENDERABLE_H__
#define __TEXT_RENDERABLE_H__

#include "../renderable.h"

#include <utility>

class FTFont;
class FTBBox;

class Text: public Renderable{
    public:
        Text(PipelineWidget& p, std::string const& text,
             std::string const& font="LiberationSans-Regular.ttf", int pt=10);
        
        virtual bool acceptsMouseEvents(){ return false; }
        virtual void draw();
        virtual BBox bbox();
    
    protected:
        typedef boost::shared_ptr<FTFont> font_ptr;
        typedef std::pair<std::string, int> face_pt_pair_t;
        static font_ptr font(face_pt_pair_t const& id);
        font_ptr font(){ return font(m_font); }

    private:
        boost::shared_ptr<FTBBox> m_bbox;
        face_pt_pair_t m_font;
        std::string m_text;
};

#endif // ndef __TEXT_RENDERABLE_H__

