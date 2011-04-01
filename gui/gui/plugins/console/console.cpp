#include "console.h"
#include "console/ui_console.h"

#include <debug/cauv_debug.h>


using namespace cauv;


#include <QTimer>

CauvConsole::CauvConsole(const char * name, QWidget * parent) : QConsole(parent, name, false) {
    setPrompt("> ");
}

bool CauvConsole::isCommandComplete(QString command){

    Q_EMIT commandReady(command);

    // don't pass it to execCommand here
    // we'll wait for the response before doing that
    return false;
}

void CauvConsole::execCommand(QString command, QString response, DebugType::e type, bool writeCommand, bool showPrompt){
    QConsole::execCommand(command, writeCommand, false);

    switch(type) {

    case DebugType::Error:
        setColor(Qt::red);
        break;

    case DebugType::Info:
    default:
        setColor(Qt::blue);
    }

    append(response);

    //Display the prompt again
    if (showPrompt)
        displayPrompt();
}


Console::Console() :
        QDockWidget(),
        m_console(new CauvConsole("Console")),
        m_counter(rand()),
        ui(new Ui::Console)
{
    ui->setupUi(this);
    this->setObjectName("GUI Console");
    setWidget(m_console);
    m_console->connect(m_console, SIGNAL(commandReady(QString)), this, SLOT(executeCommand(QString)));
    connect(this, SIGNAL(responseReceived(int,QString, DebugType::e)), this, SLOT(onResponse(int,QString,DebugType::e)));
    qRegisterMetaType<DebugType::e>("DebugType::e");

    m_docks[this] = Qt::BottomDockWidgetArea;
}

void Console::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode> node){
    CauvBasicPlugin::initialise(auv, node);
    auv->scripts.scriptResponse->onUpdate.connect(boost::bind(&Console::onResponse, this, _1));
}

Console::~Console()
{
    delete ui;
}

void Console::onResponse(int id, QString response, DebugType::e level){
    try {
        if(!m_requests.at(id).isEmpty()) {
            m_console->execCommand(m_requests[id], response, level, false, true);
            m_requests.erase(id); // we've dealt with it now
        }
    } catch (std::out_of_range) {
        error() << "Response received for an unknown request. Another GUI instance perhaps?";
    }
}

void Console::onResponse(ScriptResponse response){
    if(response.id == this->objectName().toStdString())
        Q_EMIT responseReceived(response.seq, QString::fromStdString(response.response), response.level);
}

void Console::executeCommand(QString s){
    m_requests[++m_counter] = s;

    m_auv->scripts.scriptExec->set(ScriptExecRequest(s.toStdString(), 30, this->objectName().toStdString(), m_counter));
}

const QString Console::name() const{
    return QString("Console");
}

const QList<QString> Console::getGroups() const{
    QList<QString> groups;
    groups.push_back(QString("script"));
    return groups;
}


Q_EXPORT_PLUGIN2(cauv_consoleplugin, Console)
