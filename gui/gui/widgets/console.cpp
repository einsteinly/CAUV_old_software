#include "console.h"
#include "ui_console.h"

#include "qconsole2/include/qconsole.h"

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

void CauvConsole::execCommand(QString command, QString response, bool writeCommand, bool showPrompt){
    QConsole::execCommand(command, writeCommand, false);

    append(response);

    //Display the prompt again
    if (showPrompt)
        displayPrompt();
}




Console::Console(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QDockWidget(parent),
        CauvInterfaceElement(name, auv, node),
        m_console(new CauvConsole("Console")),
        m_counter(rand()),
        ui(new Ui::Console)
{
    ui->setupUi(this);
    setWidget(m_console);
    m_console->connect(m_console, SIGNAL(commandReady(QString)), this, SLOT(executeCommand(QString)));
    connect(this, SIGNAL(responseReceived(int,QString)), this, SLOT(onResponse(int,QString)));

    auv->scripts.scriptResponse->onUpdate.connect(boost::bind(&Console::onResponse, this, _1));
}

Console::~Console()
{
    delete ui;
}

void Console::initialise(){
    m_actions->registerDockView(this, Qt::BottomDockWidgetArea);
}

void Console::onResponse(int id, QString response){
    try {
        if(!m_requests.at(id).isEmpty()) {
            m_console->execCommand(m_requests[id], response, false, true);
            m_requests.erase(id); // we've dealt with it now
        }
    } catch (std::out_of_range) {
        error() << "Response received for an unknown request. Another GUI instance perhaps?";
    }
}

void Console::onResponse(script_exec_response_t response){
    Q_EMIT responseReceived(response.seq, QString::fromStdString(response.response));
}

void Console::executeCommand(QString s){
    m_requests[++m_counter] = s;

    m_auv->scripts.scriptExec->set(script_exec_request_t(s.toStdString(), m_counter));
}

