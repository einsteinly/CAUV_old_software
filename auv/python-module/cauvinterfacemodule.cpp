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

#include "workarounds.h" // _must_ be first
#include <boost/python.hpp> // oh yes :)

#include "emit_static.h"
#include "emit_generated.h"

// omgboostpythonmagic...
BOOST_PYTHON_MODULE(cauvinterface)
{
    PyEval_InitThreads(); 

    // TODO: some sensible scopes

    // static stuff:
    emitThing();
    emitDebug();
    emitMessage();
    emitMailbox();
    emitCauvNode();
    emitAIMessageObserver();

    // generated stuff:
    emitEnums();
    emitStructs();
    emitMessages();
    emitObservers();
    emitContainers();
    emitVariants();

    // post-generated static stuff:
    emitPostGenerated();
}

