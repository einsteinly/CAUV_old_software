#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QMdiArea>

#include "../cauvinterfaceelement.h"
#include "../datastreamdragging.h"

namespace cauv {

class GraphArea : public QMdiArea, public DataStreamDropListener, public CauvInterfaceElement {
    Q_OBJECT

public:
    GraphArea(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
    virtual void initialise();
    void onStreamDropped(boost::shared_ptr<DataStream<int8_t> > stream);
    void onStreamDropped(boost::shared_ptr<DataStream<int> > stream);
    void onStreamDropped(boost::shared_ptr<DataStream<float> > stream);
    void onStreamDropped(boost::shared_ptr<DataStream<autopilot_params_t> > stream);
    void onStreamDropped(boost::shared_ptr<DataStream<floatYPR> > stream);
    void onStreamDropped(boost::shared_ptr<DataStream<uint16_t> > stream);
    void dropEvent(QDropEvent * event);
    void dragEnterEvent(QDragEnterEvent * event);
};

}

#endif // GRAPHWIDGET_H
