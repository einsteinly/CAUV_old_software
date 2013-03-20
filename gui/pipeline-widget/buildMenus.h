/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BUILD_MENUS_H__
#define __BUILD_MENUS_H__

#include <boost/shared_ptr.hpp>

#include "pwTypes.h"

namespace cauv{
namespace gui{
namespace pw{

boost::shared_ptr<Menu> buildAddNodeMenu(pw_ptr_t p);

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __BUILD_MENUS_H__

