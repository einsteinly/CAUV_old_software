/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "logview.h"
#include "logs/ui_logview.h"

#include <QTextEdit>

#include <common/cauv_utils.h>
#include <model/auv_model.h>

using namespace cauv;

LogView::LogView() :
    ui(new Ui::LogView)
{
    ui->setupUi(this);

    m_docks[this] = Qt::BottomDockWidgetArea;
}

void LogView::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode>){
    foreach(AUV::logs_map::value_type i, auv->logs){
        QTextEdit * edit = new QTextEdit();
        edit->setReadOnly(true);
        i.second->onUpdate.connect(boost::bind(&LogView::appendLog, this, edit, i.first, _1));
        ui->logTabs->addTab(edit, QString::fromStdString(i.second->getName()));
    }
}

const QString LogView::name() const{
    return QString("Logs");
}

const QList<QString> LogView::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("debug"));
    return groups;
}

void LogView::appendLog(QTextEdit *edit, DebugType::e, std::string message){
    edit->append(QString::fromStdString(message));
}

LogView::~LogView()
{
    delete ui;
}

Q_EXPORT_PLUGIN2(cauv_logsplugin, LogView)
