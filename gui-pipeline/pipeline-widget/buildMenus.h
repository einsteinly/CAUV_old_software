#ifndef __BUILD_MENUS_H__
#define __BUILD_MENUS_H__

#include <boost/shared_ptr.hpp>

class Menu;
class PipelineWidget;

typedef PipelineWidget* pw_ptr_t;
boost::shared_ptr<Menu> buildAddNodeMenu(pw_ptr_t p);

#endif // ndef __BUILD_MENUS_H__

