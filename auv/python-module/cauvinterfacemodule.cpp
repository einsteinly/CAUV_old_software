#include "workarounds.h" // _must_ be first
#include <boost/python.hpp> // oh yes :)

#include "emit_static.h"
#include "emit_generated.h"

// omgboostpythonmagic...
//#ifdef CAUV_NO_DEBUG
BOOST_PYTHON_MODULE(cauvinterface)
//#else
//BOOST_PYTHON_MODULE(cauvinterfaced)
//#endif
{
    PyEval_InitThreads(); 

    // TODO: some sensible scopes

    // static stuff:
    emitThing();
    emitDebug();
    emitMessage();
    emitMailbox();
    emitCauvNode();
    emitSpreadMessage();
    emitAIMessageObserver();

    // generated stuff:
    emitEnums();
    emitStructs();
    emitMessages();
    emitObservers();
    emitContainers();
    emitVariants();
}

