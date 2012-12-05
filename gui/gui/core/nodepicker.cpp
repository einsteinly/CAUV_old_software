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
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "nodepicker.h"
#include "ui_nodepicker.h"

#include <QtGui>
#include <boost/shared_ptr.hpp>

#include <debug/cauv_debug.h>

#include "model/nodes/numericnode.h"
#include "delegates/delegate.h"
#include "model/nodeItemModel.h"

using namespace cauv;
using namespace cauv::gui;

class EscapeFilter : public QObject {
public:
    bool eventFilter(QObject *object, QEvent *event){
        if (dynamic_cast<QLineEdit*>(object) && event->type() == QEvent::KeyPress) {
            QLineEdit * edit = (QLineEdit*)object;
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            if (keyEvent->key() == Qt::Key_Escape) {
                edit->clear();
                edit->hide();
                return true;
            } else
                return false;
        }
        return false;
    }
};


class NodePathCompleter : public QCompleter {
public:

    NodePathCompleter(QAbstractItemModel * model, QWidget * parent = NULL) :
            QCompleter(model, parent){
    }

    QStringList splitPath(const QString &path) const {
        return path.split("/");
    }

    QString pathFromIndex(const QModelIndex &index) const {
        Node *node = static_cast<Node*>(index.internalPointer());
        return QString::fromStdString(node->nodePath()).remove(QRegExp("^/"));
    }
};


NodePicker::NodePicker(boost::shared_ptr<NodeItemModel> const& root) :
     m_root(root), ui(new Ui::NodePicker())
{
    ui->setupUi(this);

    ui->view->setRootIsDecorated( true );
    ui->view->setDragEnabled(true);
    ui->view->setDropIndicatorShown(true);
    ui->view->setAcceptDrops(false);
    ui->filter->installEventFilter(new EscapeFilter());
    ui->view->setModel(root.get());
    ui->view->setRootIndex(root->indexFromNode(root->rootNode()));
    ui->view->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    ui->view->setIndentation(10);

    QHBoxLayout * layout = new QHBoxLayout(ui->filter);
    m_clearButton = new QPushButton("X");
    QFont font("Verdana", 12);
    font.setPixelSize(12);
    m_clearButton->setFont(font);
    m_clearButton->hide();
    m_clearButton->setCursor(Qt::PointingHandCursor);
    m_clearButton->connect(m_clearButton, SIGNAL(clicked()), ui->filter, SLOT(clear()));
    layout->addWidget(m_clearButton, 0, Qt::AlignLeft);
    layout->setMargin(0);
    layout->setContentsMargins(0, 0, 5, 0);

    // The list supports node filtering as there's potentially a lot of nodes
    boost::shared_ptr<NodePathFilter> pathFilter = boost::make_shared<NodePathFilter>();
    ui->view->registerListFilter(pathFilter);
    ui->filter->connect(ui->filter, SIGNAL(textChanged(QString)), pathFilter.get(), SLOT(setText(QString)));
    ui->filter->connect(ui->filter, SIGNAL(textChanged(QString)), this, SLOT(setHighlighting(QString)));
    ui->filter->connect(ui->filter, SIGNAL(editingFinished()), this, SLOT(setFilterVisisble()));

    // and auto completion of the path filter
    QCompleter * completer = new NodePathCompleter(ui->view->model());
    completer->setCaseSensitivity(Qt::CaseInsensitive);
    ui->filter->setCompleter(completer);
    ui->filter->hide();

    // if the tree changes whilst we're being filtered we need to re-apply filters
    root->connect(root->rootNode().get(), SIGNAL(structureChanged()), ui->view, SLOT(applyFilters()));

    // if the list is in focus (but keystrokes are not swalled by an edit box) then
    // redirect focus so the filter gets the key events
    ui->view->connect(ui->view, SIGNAL(onKeyPressed(QKeyEvent*)), this, SLOT(redirectKeyboardFocus(QKeyEvent*)));

}

void NodePicker::setHighlighting(QString text){
    if(text.isEmpty()){
        m_clearButton->hide();
    } else{
        m_clearButton->show();
    }
}

void NodePicker::setFilterVisisble(){
    if(ui->filter->text().isEmpty()){
        ui->filter->hide();
    } else{
        ui->filter->show();
    }
}

void NodePicker::redirectKeyboardFocus(QKeyEvent* event){
    if(event->key() == Qt::Key_Escape) {
        ui->filter->setText("");
        ui->filter->hide();
        return;
    }
    if(!event->text().isEmpty() && event->text().at(0).isLetterOrNumber()) {
        ui->filter->setText("");
        ui->filter->show();
        ui->filter->setFocus();
        ui->filter->setText(event->text().simplified());
    }
}

void NodePicker::registerListFilter(boost::shared_ptr<NodeFilterInterface> const& filter){
   ui->view->registerListFilter(filter);
}

void NodePicker::registerDelegate(node_type nodeType,
                                  boost::shared_ptr<AbstractNodeDelegate> delegate){
    ui->view->registerDelegate(nodeType, delegate);
}

NodePicker::~NodePicker(){
    delete ui;
}

