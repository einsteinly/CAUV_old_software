#ifdef USE_MARBLE

#ifndef MAPVIEW_H
#define MAPVIEW_H

#include <QWidget>

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <QTimer>

#include "cauvinterfaceelement.h"

#include <model/auv_model.h>
#include <marble/PositionProviderPlugin.h>

namespace Ui {
    class MapView;
}

namespace Marble {
    class MarbleWidget;
}

namespace cauv {

    class CauvPositionProvider : public Marble::PositionProviderPlugin {

        Q_OBJECT

    public:

        CauvPositionProvider() : Marble::PositionProviderPlugin(){

                QTimer * timer = new QTimer();
                timer->connect(timer, SIGNAL(timeout()), this, SLOT(test()));
                timer->setSingleShot(false);
                timer->start(100);
        }


        Marble::PositionProviderPlugin * newInstance() const {
            return new CauvPositionProvider();
        }

        qreal speed() const {
            return 0;
        }

        qreal direction() const {
            return 0;
        }

        Marble::PositionProviderStatus status() const {
            return Marble::PositionProviderStatusAvailable;
        }

        Marble::GeoDataCoordinates position() const {
            return Marble::GeoDataCoordinates(0, 0, 0, Marble::GeoDataCoordinates::Degree);
        }

        Marble::GeoDataAccuracy accuracy() const {
            Marble::GeoDataAccuracy accuracy = Marble::GeoDataAccuracy();
            accuracy.horizontal = 1;
            accuracy.vertical = 1;
            accuracy.level = Marble::GeoDataAccuracy::Detailed;
            return accuracy;
        }

        virtual QString name() const {
            return QString("CAUV Postion Provider");
        }

        virtual QString guiString() const {
            return name();
        }

        virtual QString nameId() const {
            return QString("CauvPositionProvider");
        }

        virtual QString description() const {
            return name();
        }

        virtual QIcon icon() const {
            return QIcon();
        }

        virtual void initialize(){
        }

        virtual bool isInitialized() const{
            return true;
        }

        public Q_SLOTS:
            void test(){
                Q_EMIT positionChanged(Marble::GeoDataCoordinates(rand(), rand(), 0), accuracy());
            }

    };


    class MapView : public QWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        MapView(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual ~MapView();

        virtual void initialise();

    protected:
        Marble::MarbleWidget *m_marbleWidget;

    private:
        Ui::MapView *ui;
    };

} // namesapce cauv

#endif // MAPVIEW_H

#endif // USE_MARBLE
