#include "console.h"
#include "ui_console.h"

#include "qconsole2/include/qconsole.h"

using namespace cauv;

Console::Console(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
    QDockWidget(parent),
    CauvInterfaceElement(name, auv, node),
    m_console(new QConsole(NULL, "console", false)),
    ui(new Ui::Console)
{
    ui->setupUi(this);
    m_console->setPrompt(" > ", false);
    setWidget(m_console);
}

Console::~Console()
{
    delete ui;
}

void Console::initialise(){
    m_actions->registerDockView(this, Qt::BottomDockWidgetArea);
}
