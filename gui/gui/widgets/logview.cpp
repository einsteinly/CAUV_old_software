#include "logview.h"
#include "ui_logview.h"

#include <QTextEdit>

#include <model/auv_model.h>

using namespace cauv;

LogView::LogView(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
    QDockWidget(parent),
    CauvInterfaceElement(name, auv, node),
    ui(new Ui::LogView)
{
    ui->setupUi(this);

    // replace the title bar as it takes up too much room.
    // TODO: put something useful here instead?
    //QWidget* titleWidget = new QWidget(this);
    //this->setTitleBarWidget( titleWidget );

    foreach(AUV::logs_map::value_type i, auv->logs){
        QTextEdit * edit = new QTextEdit();
        edit->setReadOnly(true);
        i.second->onUpdate.connect(boost::bind(&LogView::appendLog, this, edit, i.first, _1));
        ui->logTabs->addTab(edit, QString::fromStdString(i.second->getName()));
    }
}

void LogView::initialise(){
    m_actions->registerDockView(this, Qt::BottomDockWidgetArea);
}

void LogView::appendLog(QTextEdit *edit, DebugType::e, std::string message){
    edit->append(QString::fromStdString(message));
}

LogView::~LogView()
{
    delete ui;
}
