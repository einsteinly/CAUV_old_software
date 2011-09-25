#include "fNode.h"

#include <set>

#include <common/cauv_utils.h>

#include <liquid/button.h>
#include <liquid/nodeHeader.h>

#include "style.h"
#include "nodeInput.h"

using cauv::gui::FNode;
using namespace liquid;

FNode::FNode(Manager& m, QGraphicsItem *parent)
    : liquid::LiquidNode(F_Node_Style, parent),
      ManagedElement(m){

    Button *collapsebutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"), NULL, this
    );
    m_header->addButton("collapse", collapsebutton);

    Button *execbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), NULL, this
    );
    m_header->addButton("exec", execbutton);
    
    Button *dupbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/dup_button"), NULL, this
    );
    m_header->addButton("duplicate", dupbutton);
    

    setSize(QSizeF(104,130));

    //!!!
    addItem(new NodeInput(m_style, NodeIOType::Image, true, this));
    addItem(new NodeInput(m_style, NodeIOType::Image, false, this));
    addItem(new NodeInput(m_style, NodeIOType::Parameter, true, this));
    addItem(new NodeInput(m_style, NodeIOType::Parameter, false, this));
}

