/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __TEXT_RENDERABLE_H__
#define __TEXT_RENDERABLE_H__

#include "../renderable.h"

#include <utility>
#include <string>

class FTFont;

namespace cauv{
namespace gui{
namespace pw{

class Text: public Renderable, public std::string{
    public:
        Text(container_ptr_t c, std::string const& text,
             std::string const& font="sans", int pt=12);
        virtual ~Text(){ }

        virtual bool acceptsMouseEvents(){ return false; }
        virtual void draw(drawtype_e::e flags);
        virtual BBox bbox();
        virtual void updateBbox();

        void colour(Colour const& c);
    
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


        // FIXME: texture based fonts don't seem to play well with OS X,
        // something to do with texture resource management, probably
        font_ptr font(){ return font(m_font); }
        font_ptr bboxFont(){ return bboxFont(m_font); }

    private:
        BBox m_bbox;
        face_pt_pair_t m_font;
        Colour m_colour;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __TEXT_RENDERABLE_H__

