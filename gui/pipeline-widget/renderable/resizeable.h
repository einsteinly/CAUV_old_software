/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __RESIZEABLE_RENDERABLE_H__
#define __RESIZEABLE_RENDERABLE_H__

#include "../renderable.h"

namespace cauv{
namespace gui{
namespace pw{

class Resizeable: public Renderable{
    public:
        Resizeable(container_ptr_t c,
                   BBox const& initial_size = BBox(10, 10),
                   BBox const& min_size = BBox(0, 0),
                   BBox const& max_size = BBox(0, 0),
                   double const& fixed_aspect = -1);
        virtual ~Resizeable(){ }

        virtual void mouseMoveEvent(MouseEvent const&);
        virtual bool mousePressEvent(MouseEvent const&);
        virtual void mouseReleaseEvent(MouseEvent const&);

        virtual BBox bbox();

    protected:
        void drawHandle();
        void aspect(double const& w_over_h);

        BBox m_bbox;

    private:
        Point m_click_offset;
        bool m_resizing;
        
        BBox m_min_bbox;
        BBox m_max_bbox;

        double m_aspect;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __RESIZEABLE_RENDERABLE_H__

