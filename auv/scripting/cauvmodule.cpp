#include "workarounds.h" // _must_ be first
#include <boost/python.hpp> // oh yes :)

#include "emit_static.h"
#include "emit_generated.h"

// omgboostpythonmagic...
BOOST_PYTHON_MODULE(cauv)
{
    PyEval_InitThreads(); 

    // TODO: some sensible scopes

    // static stuff:
    emitThing();
    emitMessage();
    emitMailbox();
    emitCauvNode();
    emitSpreadMessage();

    // generated stuff:
    emitEnums();
    emitStructs();
    emitMessages();
    emitObservers();
}

