#ifndef __BUILD_MENUS_H__
#define __BUILD_MENUS_H__

#include <boost/shared_ptr.hpp>

class Renderable;
class PipelineWidget;

boost::shared_ptr<Renderable> buildAddNodeMenu(PipelineWidget&);

#endif // ndef __BUILD_MENUS_H__

