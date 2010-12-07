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
}

