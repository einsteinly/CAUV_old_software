/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
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

