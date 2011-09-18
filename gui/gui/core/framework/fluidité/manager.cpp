#include "manager.h"

using namespace cauv;
using namespace cauv::gui;

Manager::Manager(QGraphicsScene *scene, CauvNode *node)
    : m_scene(scene),
      m_node(node){
    //m_node->addMessageObserver(this);
}

