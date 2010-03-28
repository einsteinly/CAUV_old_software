#ifndef __BUILD_MENUS_H__
#define __BUILD_MENUS_H__

#include <boost/shared_ptr.hpp>

class Menu;
class PipelineWidget;

boost::shared_ptr<Menu> buildAddNodeMenu(PipelineWidget&);

#endif // ndef __BUILD_MENUS_H__

