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


#ifndef __LIQUID_FORWARD_H__
#define __LIQUID_FORWARD_H__

namespace liquid{

// arc.h
class Arc;

// arcSink.h
class AbstractArcSink;
class ArcSink;

// arcSource.h
class ArcSourceDelegate;
class AbstractArcSource;
class ArcSource;

// button.h
class Button;

// connectionSink.h
class ConnectionSink;
class RejectingConnectionSink;

// ephemeralArcEnd.h
class EphemeralArcEnd;

// node.h
class LiquidNode;

// nodeHeader.h
class NodeHeader;

// requiresCutout.h
class RequiresCutout;

// resize.h
class ResizeHandle;

// style.h
struct ArcStyle;
struct TextStyle;
struct CutoutStyle;
struct NodeStyle;

// view.h
class LiquidView;

} // namespace liquid

#endif // ndef __LIQUID_FORWARD_H__

