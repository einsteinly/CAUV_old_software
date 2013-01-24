/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

// shadow.h
class Shadow;

// proxyWidget.h
class ProxyWidget;

// itemFridge.h
template<typename T>
class ItemFridge;

} // namespace liquid

#endif // ndef __LIQUID_FORWARD_H__

