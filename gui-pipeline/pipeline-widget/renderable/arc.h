#ifndef __ARC_RENDERABLE_H__
#define __ARC_RENDERABLE_H__

#include <common/messages.h>

#include "../renderable.h"
#include "../pwTypes.h"

namespace pw{

// Arcs draw their stuff in top level coordinates, so they can't be used in
// containers
class Arc: public Renderable{
    public:
        Arc(container_ptr_t c, renderable_wkptr_t src, renderable_wkptr_t dst);
        virtual ~Arc(){ }

        virtual void draw(bool picking); 
        virtual bool acceptsMouseEvents();
        
        // TODO: eww, introduces dependency on messages.h
        NodeOutput from(); 
        NodeInput to();
        
        renderable_wkptr_t m_src;
        renderable_wkptr_t m_dst;
        bool m_hanging;
};

} // namespace pw


#endif // ndef __ARC_RENDERABLE_H__

