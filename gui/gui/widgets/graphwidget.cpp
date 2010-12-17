#include "graphwidget.h"
#include "ui_graphwidget.h"

#include <QListWidgetItem>

GraphWidget::GraphWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent) :
    QDockWidget(parent),
    CauvInterfaceElement(name, auv)
{
    setupUi(this);

    dataStreams->setRootIsDecorated( true );

      //QListWidgetItem *root = new QListWidgetItem("root", dataStreams );
      /*
      QListViewItem *a = new QListViewItem( root, "A" );
      QListViewItem *b = new QListViewItem( root, "B" );
      QListViewItem *c = new QListViewItem( root, "C" );

      new QListViewItem( a, "foo", "1", "2", "3" );
      new QListViewItem( a, "bar", "i", "ii", "iii" );
      new QListViewItem( a, "baz", "a", "b", "c" );

      new QListViewItem( b, "foo", "1", "2", "3" );
      new QListViewItem( b, "bar", "i", "ii", "iii" );
      new QListViewItem( b, "baz", "a", "b", "c" );

      new QListViewItem( c, "foo", "1", "2", "3" );
      new QListViewItem( c, "bar", "i", "ii", "iii" );
      new QListViewItem( c, "baz", "a", "b", "c" );*/


}

void GraphWidget::initialise(){
    m_actions->registerDockView(this, Qt::RightDockWidgetArea);
}
