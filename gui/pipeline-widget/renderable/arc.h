#ifndef __ARC_RENDERABLE_H__
#define __ARC_RENDERABLE_H__

#include <generated/types/NodeInput.h>
#include <generated/types/NodeOutput.h>

#include "../renderable.h"
#include "../pwTypes.h"

namespace cauv{
namespace pw{

// Arcs draw their stuff in top level coordinates, so they can't be used in
// containers
class Arc: public Renderable{
    public:
        Arc(container_ptr_t c, renderable_wkptr_t src, renderable_wkptr_t dst);
        virtual ~Arc(){ }

        virtual void draw(drawtype_e::e flags); 
        virtual bool acceptsMouseEvents();
        
        NodeOutput from(); 
        NodeInput to();

        renderable_ptr_t fromOutput();
        renderable_ptr_t toInput();
        
        renderable_wkptr_t m_src;
        renderable_wkptr_t m_dst;
        bool m_hanging;
};

} // namespace pw
} // namespace cauv

#endif // ndef __ARC_RENDERABLE_H__

