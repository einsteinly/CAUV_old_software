#ifndef MAPVIEW_H
#define MAPVIEW_H

#include <QWidget>
#include <QTimer>

#include <math.h>
#include <boost/make_shared.hpp>
#include <debug/cauv_debug.h>
#include <model/auv_model.h>
#include <gui/core/cauvbasicplugin.h>

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

        CauvPositionProvider(boost::shared_ptr<AUV> auv) : Marble::PositionProviderPlugin(), m_auv(auv) {
            qRegisterMetaType<Marble::GeoDataAccuracy>("GeoDataAccuracy");
            m_auv->sensors.location->onUpdate.connect(boost::bind(&CauvPositionProvider::onPositionUpdate, this, _1));
        }


        Marble::PositionProviderPlugin * newInstance() const {
            throw new std::exception();
        }

        qreal speed() const {
            float x = m_auv->sensors.speed->latest().x;
            float y = m_auv->sensors.speed->latest().y;
            return sqrt(x*x*y*y);
        }

        qreal direction() const {
            return m_auv->sensors.orientation->yaw->latest();
        }

        Marble::PositionProviderStatus status() const {
            return Marble::PositionProviderStatusAvailable;
        }

        Marble::GeoDataCoordinates position() const {
            qreal lng = m_auv->sensors.location->latest().longitude();
            qreal lat = m_auv->sensors.location->latest().latitude();
            qreal alt = 100;//m_auv->sensors.location->latest().altitude();

            info() << lat << lng << alt;

            return Marble::GeoDataCoordinates(lng, lat, alt, Marble::GeoDataCoordinates::Degree);
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
            void onPositionUpdate(Location){
                info() << "Position upadted";
                Q_EMIT positionChanged(position(), accuracy());
            }

        protected:
            boost::shared_ptr<AUV> m_auv;

    };


    class MapView : public QWidget, public CauvBasicPlugin {
        Q_OBJECT
        Q_INTERFACES(cauv::CauvInterfacePlugin)

    public:
        MapView();
        virtual ~MapView();

        void initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode>);

        void updateHomePoint();

        virtual const QString name() const;
        virtual const QList<QString> getGroups() const;

    protected:
        Marble::MarbleWidget *m_marbleWidget;

    private:
        Ui::MapView *ui;
    };

} // namesapce cauv

#endif // MAPVIEW_H
