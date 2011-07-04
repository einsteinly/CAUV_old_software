#ifndef DATASTREAMDISPLAYS_H
#define DATASTREAMDISPLAYS_H

#include <QDockWidget>
#include <QVariant>
#include <QTreeWidget>
#include <QMdiArea>

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <gui/core/cauvbasicplugin.h>
#include <gui/core/datastreamdragging.h>
#include <gui/core/model/model.h>

namespace Ui {
    class DataStreamPicker;
}


namespace cauv {
    namespace gui {


        /**
      * DataStreamDisplayArea - accepts data stream drops, adding them to its mdi area as it gets them
      *
      * @author Andy Pritchard
      */
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



        /**
      * Interface integration class.
      *
      * @author Andy Pritchard
      */
        class DataStreamPicker : public QDockWidget, public CauvBasicPlugin{
            Q_OBJECT
            Q_INTERFACES(cauv::gui::CauvInterfacePlugin)


        public:
                    DataStreamPicker();
            virtual ~DataStreamPicker();

            virtual const QString name() const;
            virtual const QList<QString> getGroups() const;
            virtual void initialise(boost::shared_ptr<AUV>auv, boost::shared_ptr<CauvNode>node);

        private:
            Ui::DataStreamPicker *ui;
        };

    } // namespace gui
} // namespace cauv


/**
  * The QWidget subclass where the model data is displayed.
  *
  * It's outside the cauv namespace because its used as promoted widget in a ui file
  * TODO: find a way around this
  *
  * @author Andy Pritchard
  */
class DataStreamList : public QTreeWidget, public cauv::gui::NodeDragSource {
    Q_OBJECT
public:
    DataStreamList(QWidget * parent);
    
    boost::shared_ptr<std::vector<boost::shared_ptr<cauv::gui::NodeBase> > > getDroppedNodes();
    
private Q_SLOTS:
    void editStarted(QTreeWidgetItem* item, int column);
    void itemEdited(QTreeWidgetItem* item, int column);
};

#endif // DATASTREAMDISPLAYS_H
