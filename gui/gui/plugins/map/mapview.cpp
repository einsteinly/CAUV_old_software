#include "mapview.h"
#include "map/ui_mapview.h"

#include <model/auv_model.h>
#include <marble/MarbleWidget.h>
#include <marble/MarbleModel.h>
#include <marble/PositionTracking.h>

using namespace cauv;

MapView::MapView() :
        ui(new Ui::MapView)
{
    ui->setupUi(this);

    m_marbleWidget = new Marble::MarbleWidget(this);
    m_marbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    layout()->addWidget(m_marbleWidget);
    m_marbleWidget->setShowGps(true);

    Marble::MarbleModel * model = m_marbleWidget->model();

    Marble::PositionTracking * tracker = model->positionTracking();
    tracker->setPositionProviderPlugin(new CauvPositionProvider());
    tracker->setTrackVisible(true);

    m_tabs.append(m_marbleWidget);

}

MapView::~MapView()
{
    delete ui;
}

const QString MapView::name() const {
    return QString("Map");
}

const QList<QString> MapView::getGroups() const {
    QList<QString> groups;
    groups.push_back(QString("telemetry"));
    return groups;
}

Q_EXPORT_PLUGIN2(cauv_mapplugin, MapView)

