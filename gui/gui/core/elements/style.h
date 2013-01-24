/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_ELEMENT_STYLE_H__
#define __CAUV_GUI_ELEMENT_STYLE_H__

#include <liquid/style.h>

namespace cauv{
namespace gui{

const liquid::ArcStyle& Image_Arc_Style();

const liquid::ArcStyle& Param_Arc_Style();

const liquid::CutoutStyle& Required_Image_Input();
const liquid::CutoutStyle& Required_Param_Input();
const liquid::CutoutStyle& Optional_Image_Input();
const liquid::CutoutStyle& Optional_Param_Input();

const liquid::NodeStyle &F_Node_Style();
const liquid::NodeStyle &AI_Node_Style();
const liquid::NodeStyle &Graph_Node_Style();

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_ELEMENT_STYLE_H__

