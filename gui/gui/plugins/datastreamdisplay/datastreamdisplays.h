#ifndef DATASTREAMDISPLAYS_H
#define DATASTREAMDISPLAYS_H

#include <QDockWidget>
#include <QVariant>
#include <QTreeWidget>
#include <QMdiArea>
#include <QKeyEvent>

#include <debug/cauv_debug.h>

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/datastreamdragging.h>


namespace Ui {
    class DataStreamPicker;
}


namespace cauv {
    namespace gui {


        class DataStreamDisplayArea : public QMdiArea, public NodeDropListener {
            Q_OBJECT

        public:
            DataStreamDisplayArea(QWidget * parent = 0);
            void onNodeDropped(boost::shared_ptr<NumericNode> );
            void onNodeDropped(boost::shared_ptr<ImageNode> );
            void onNodeDropped(boost::shared_ptr<FloatYPRNode> );
            void onNodeDropped(boost::shared_ptr<FloatXYZNode> );
            void onNodeDropped(boost::shared_ptr<GroupingNode> );
            void dropEvent(QDropEvent * event);
            void dragEnterEvent(QDragEnterEvent * event);
            void addWindow(boost::shared_ptr<QWidget> content);
        };




        class DataStreamPicker : public QDockWidget, public CauvBasicPlugin{
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)

        public:
            DataStreamPicker();
            virtual ~DataStreamPicker();

            virtual const QString name() const;
            virtual const QList<QString> getGroups() const;
            virtual void initialise(boost::shared_ptr<AUV>auv, boost::shared_ptr<CauvNode>node);

        protected Q_SLOTS:
            void filterItems(QString);
            void redirectKeyboardFocus(QKeyEvent* key);

        private:
            Ui::DataStreamPicker *ui;
        };

    } // namespace gui
} // namespace cauv


/**
  * The QWidget subclass where the model data is displayed.
  *
  * It's outside the cauv namespace because its used as promoted widget in a ui file
  * @todo: find a way around this
  *
  * @author Andy Pritchard
  */
class DataStreamList : public QTreeWidget, public cauv::gui::NodeDragSource {
    Q_OBJECT
public:
    DataStreamList(QWidget * parent);
    
    boost::shared_ptr<std::vector<boost::shared_ptr<cauv::gui::NodeBase> > > getDroppedNodes();
    
    void keyPressEvent(QKeyEvent *event);

private Q_SLOTS:
    void editStarted(QTreeWidgetItem* item, int column);
    void itemEdited(QTreeWidgetItem* item, int column);

Q_SIGNALS:
    void onKeyPressed(QKeyEvent *event);
};

#endif // DATASTREAMDISPLAYS_H
