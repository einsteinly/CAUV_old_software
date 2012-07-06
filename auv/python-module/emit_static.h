/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef EMIT_STATIC_H
#define EMIT_STATIC_H


void emitThing(); // test stuff

void emitDebug();
void emitMailbox();
void emitMessage();
void emitCauvNode();
void emitAIMessageObserver();

void emitSpreadMessage(); // not really needed?

void emitPostGenerated();

#endif // EMIT_STATIC_H

