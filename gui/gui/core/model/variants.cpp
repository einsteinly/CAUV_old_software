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

#include "variants.h"

using namespace cauv;
using namespace cauv::gui;


Colour qColorToColour(TypedQColor colour) {
    switch (colour.colorType()) {
    case ColourType::RGB:
        //return Colour::fromRGB(colour.redF(), colour.greenF(), colour.blueF());
    case ColourType::ARGB:
        //return Colour::fromRGBA(colour.redF(), colour.greenF(), colour.blueF(), colour.alphaF());
    case ColourType::BGR:
        //return Colour::fromBGR(colour.redF(), colour.greenF(), colour.blueF());
    case ColourType::BGRA:
        //return Colour::fromBGRA(colour.redF(), colour.greenF(), colour.blueF(), colour.alphaF());
    case ColourType::Grey:
        break;
    default:
        break;
        //return Colour::fromGrey(colour.value());
    }
    return Colour();
}

TypedQColor colorToQColour(Colour const& colour) {
    TypedQColor c;
/*
    switch(colour.type){
        case ColourType::Grey:
            c = TypedQColor::fromHsv(0, 0, colour.grey());

        case ColourType::RGB:
        case ColourType::ARGB:
        case ColourType::BGR:
        case ColourType::BGRA: {
            c = TypedQColor::fromHsv(0, 0, colour.grey());

        default:
            warning() << "colorToQColour() - Unknown colour format";
            c = TypedQColor::fromHsv(0, 0, colour.grey());
*/
    c.setColorType(colour.type);
    return c;
}
