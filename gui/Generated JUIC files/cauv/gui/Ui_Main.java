/********************************************************************************
** Form generated from reading ui file 'Main.jui'
**
** Created: Sun 27. Jun 22:43:57 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_Main implements com.trolltech.qt.QUiForm<QMainWindow>
{
    public QAction actionSettings;
    public QAction actionMoo;
    public QAction actionPahh_You_wish;
    public QWidget centralwidget;
    public QVBoxLayout verticalLayout;
    public QWidget connectionBar;
    public QHBoxLayout horizontalLayout;
    public QLabel label_3;
    public QWidget connectPanel;
    public QHBoxLayout horizontalLayout_6;
    public QLineEdit address;
    public QSpinBox port;
    public QPushButton connectButton;
    public QLabel errorMessage;
    public QLabel controlLED;
    public QLabel imageProcLED;
    public QLabel aiLED;
    public QWidget widget;
    public QHBoxLayout horizontalLayout_8;
    public QWidget widget_2;
    public QVBoxLayout verticalLayout_2;
    public QPushButton backButton;
    public QStackedWidget informationStack;
    public QWidget page;
    public QGridLayout gridLayout;
    public QSpacerItem horizontalSpacer_2;
    public QSpacerItem horizontalSpacer_3;
    public QSpacerItem verticalSpacer;
    public QSpacerItem verticalSpacer_2;
    public QGridLayout iconLayout;
    public QWidget page_2;
    public QStatusBar statusbar;
    public QDockWidget dockWidget;
    public QWidget dockWidgetContents_2;
    public QHBoxLayout horizontalLayout_2;
    public QTabWidget tabWidget;
    public QWidget tab_4;
    public QHBoxLayout horizontalLayout_7;
    public QTextEdit all;
    public QWidget tab;
    public QHBoxLayout horizontalLayout_3;
    public QTextEdit errors;
    public QWidget tab_2;
    public QHBoxLayout horizontalLayout_4;
    public QTextEdit warnings;
    public QWidget tab_3;
    public QHBoxLayout horizontalLayout_5;
    public QTextEdit traces;

    public Ui_Main() { super(); }

    public void setupUi(QMainWindow Main)
    {
        Main.setObjectName("Main");
        Main.resize(new QSize(827, 623).expandedTo(Main.minimumSizeHint()));
        Main.setWindowIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icon.png")));
        actionSettings = new QAction(Main);
        actionSettings.setObjectName("actionSettings");
        actionMoo = new QAction(Main);
        actionMoo.setObjectName("actionMoo");
        actionPahh_You_wish = new QAction(Main);
        actionPahh_You_wish.setObjectName("actionPahh_You_wish");
        centralwidget = new QWidget(Main);
        centralwidget.setObjectName("centralwidget");
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout.setSpacing(0);
        verticalLayout.setMargin(0);
        verticalLayout.setObjectName("verticalLayout");
        connectionBar = new QWidget(centralwidget);
        connectionBar.setObjectName("connectionBar");
        connectionBar.setMaximumSize(new QSize(16777215, 30));
        QPalette palette= new QPalette();
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        connectionBar.setPalette(palette);
        connectionBar.setAutoFillBackground(true);
        horizontalLayout = new QHBoxLayout(connectionBar);
        horizontalLayout.setObjectName("horizontalLayout");
        horizontalLayout.setContentsMargins(6, 2, 2, 2);
        label_3 = new QLabel(connectionBar);
        label_3.setObjectName("label_3");

        horizontalLayout.addWidget(label_3);

        connectPanel = new QWidget(connectionBar);
        connectPanel.setObjectName("connectPanel");
        horizontalLayout_6 = new QHBoxLayout(connectPanel);
        horizontalLayout_6.setMargin(0);
        horizontalLayout_6.setObjectName("horizontalLayout_6");
        address = new QLineEdit(connectPanel);
        address.setObjectName("address");
        QSizePolicy sizePolicy = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Fixed, com.trolltech.qt.gui.QSizePolicy.Policy.Fixed);
        sizePolicy.setHorizontalStretch((byte)0);
        sizePolicy.setVerticalStretch((byte)0);
        sizePolicy.setHeightForWidth(address.sizePolicy().hasHeightForWidth());
        address.setSizePolicy(sizePolicy);
        address.setMaximumSize(new QSize(16777215, 22));
        QPalette palette1= new QPalette();
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(127, 127, 127));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(127, 127, 127));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(127, 127, 127));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        address.setPalette(palette1);
        address.setFrame(false);

        horizontalLayout_6.addWidget(address);

        port = new QSpinBox(connectPanel);
        port.setObjectName("port");
        port.setMaximumSize(new QSize(16777215, 22));
        QPalette palette2= new QPalette();
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(127, 127, 127));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(127, 127, 127));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(127, 127, 127));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        port.setPalette(palette2);
        port.setFrame(false);
        port.setMinimum(0);
        port.setMaximum(99999999);

        horizontalLayout_6.addWidget(port);

        connectButton = new QPushButton(connectPanel);
        connectButton.setObjectName("connectButton");
        QPalette palette3= new QPalette();
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(127, 127, 127));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(127, 127, 127));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(127, 127, 127));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        connectButton.setPalette(palette3);
        connectButton.setFlat(false);

        horizontalLayout_6.addWidget(connectButton);


        horizontalLayout.addWidget(connectPanel);

        errorMessage = new QLabel(connectionBar);
        errorMessage.setObjectName("errorMessage");
        QSizePolicy sizePolicy1 = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Preferred);
        sizePolicy1.setHorizontalStretch((byte)0);
        sizePolicy1.setVerticalStretch((byte)0);
        sizePolicy1.setHeightForWidth(errorMessage.sizePolicy().hasHeightForWidth());
        errorMessage.setSizePolicy(sizePolicy1);
        errorMessage.setStyleSheet("#errorMessage{\n"+
"	font-size: 11px;\n"+
"	color: rgb(255, 0, 0);\n"+
"	font-weight: bold;\n"+
"	text-align: center;\n"+
"}");
        errorMessage.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignLeft,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));
        errorMessage.setWordWrap(true);
        errorMessage.setIndent(0);

        horizontalLayout.addWidget(errorMessage);

        controlLED = new QLabel(connectionBar);
        controlLED.setObjectName("controlLED");
        controlLED.setMinimumSize(new QSize(20, 20));
        controlLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        horizontalLayout.addWidget(controlLED);

        imageProcLED = new QLabel(connectionBar);
        imageProcLED.setObjectName("imageProcLED");
        imageProcLED.setMinimumSize(new QSize(20, 20));
        imageProcLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        horizontalLayout.addWidget(imageProcLED);

        aiLED = new QLabel(connectionBar);
        aiLED.setObjectName("aiLED");
        aiLED.setMinimumSize(new QSize(20, 20));
        aiLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        horizontalLayout.addWidget(aiLED);


        verticalLayout.addWidget(connectionBar);

        widget = new QWidget(centralwidget);
        widget.setObjectName("widget");
        horizontalLayout_8 = new QHBoxLayout(widget);
        horizontalLayout_8.setSpacing(0);
        horizontalLayout_8.setMargin(0);
        horizontalLayout_8.setObjectName("horizontalLayout_8");
        widget_2 = new QWidget(widget);
        widget_2.setObjectName("widget_2");
        QSizePolicy sizePolicy2 = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Ignored);
        sizePolicy2.setHorizontalStretch((byte)0);
        sizePolicy2.setVerticalStretch((byte)0);
        sizePolicy2.setHeightForWidth(widget_2.sizePolicy().hasHeightForWidth());
        widget_2.setSizePolicy(sizePolicy2);
        widget_2.setMinimumSize(new QSize(0, 0));
        verticalLayout_2 = new QVBoxLayout(widget_2);
        verticalLayout_2.setMargin(0);
        verticalLayout_2.setObjectName("verticalLayout_2");
        backButton = new QPushButton(widget_2);
        backButton.setObjectName("backButton");
        QSizePolicy sizePolicy3 = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Fixed, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);
        sizePolicy3.setHorizontalStretch((byte)0);
        sizePolicy3.setVerticalStretch((byte)0);
        sizePolicy3.setHeightForWidth(backButton.sizePolicy().hasHeightForWidth());
        backButton.setSizePolicy(sizePolicy3);
        backButton.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));
        backButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/arrow-left.png")));
        backButton.setIconSize(new QSize(12, 12));

        verticalLayout_2.addWidget(backButton);


        horizontalLayout_8.addWidget(widget_2);

        informationStack = new QStackedWidget(widget);
        informationStack.setObjectName("informationStack");
        page = new QWidget();
        page.setObjectName("page");
        page.setAutoFillBackground(true);
        gridLayout = new QGridLayout(page);
        gridLayout.setSpacing(0);
        gridLayout.setMargin(0);
        gridLayout.setObjectName("gridLayout");
        horizontalSpacer_2 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout.addItem(horizontalSpacer_2, 1, 0, 1, 1);

        horizontalSpacer_3 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout.addItem(horizontalSpacer_3, 1, 3, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout.addItem(verticalSpacer, 2, 2, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout.addItem(verticalSpacer_2, 0, 2, 1, 1);

        iconLayout = new QGridLayout();
        iconLayout.setSpacing(15);
        iconLayout.setObjectName("iconLayout");

        gridLayout.addLayout(iconLayout, 1, 2, 1, 1);

        informationStack.addWidget(page);
        page_2 = new QWidget();
        page_2.setObjectName("page_2");
        informationStack.addWidget(page_2);

        horizontalLayout_8.addWidget(informationStack);


        verticalLayout.addWidget(widget);

        Main.setCentralWidget(centralwidget);
        statusbar = new QStatusBar(Main);
        statusbar.setObjectName("statusbar");
        Main.setStatusBar(statusbar);
        dockWidget = new QDockWidget(Main);
        dockWidget.setObjectName("dockWidget");
        dockWidget.setMinimumSize(new QSize(145, 139));
        dockWidget.setFloating(false);
        dockWidget.setFeatures(com.trolltech.qt.gui.QDockWidget.DockWidgetFeature.createQFlags(com.trolltech.qt.gui.QDockWidget.DockWidgetFeature.DockWidgetFloatable,com.trolltech.qt.gui.QDockWidget.DockWidgetFeature.DockWidgetMovable));
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2.setObjectName("dockWidgetContents_2");
        horizontalLayout_2 = new QHBoxLayout(dockWidgetContents_2);
        horizontalLayout_2.setSpacing(0);
        horizontalLayout_2.setObjectName("horizontalLayout_2");
        horizontalLayout_2.setContentsMargins(2, 0, 2, 0);
        tabWidget = new QTabWidget(dockWidgetContents_2);
        tabWidget.setObjectName("tabWidget");
        tab_4 = new QWidget();
        tab_4.setObjectName("tab_4");
        horizontalLayout_7 = new QHBoxLayout(tab_4);
        horizontalLayout_7.setMargin(0);
        horizontalLayout_7.setObjectName("horizontalLayout_7");
        all = new QTextEdit(tab_4);
        all.setObjectName("all");
        all.setFrameShape(com.trolltech.qt.gui.QFrame.Shape.Box);
        all.setFrameShadow(com.trolltech.qt.gui.QFrame.Shadow.Plain);
        all.setLineWidth(0);
        all.setVerticalScrollBarPolicy(com.trolltech.qt.core.Qt.ScrollBarPolicy.ScrollBarAlwaysOn);
        all.setReadOnly(true);

        horizontalLayout_7.addWidget(all);

        tabWidget.addTab(tab_4, new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/comments.png")), com.trolltech.qt.core.QCoreApplication.translate("Main", "All", null));
        tab = new QWidget();
        tab.setObjectName("tab");
        horizontalLayout_3 = new QHBoxLayout(tab);
        horizontalLayout_3.setMargin(0);
        horizontalLayout_3.setObjectName("horizontalLayout_3");
        errors = new QTextEdit(tab);
        errors.setObjectName("errors");
        errors.setFrameShape(com.trolltech.qt.gui.QFrame.Shape.Box);
        errors.setFrameShadow(com.trolltech.qt.gui.QFrame.Shadow.Plain);
        errors.setLineWidth(0);
        errors.setVerticalScrollBarPolicy(com.trolltech.qt.core.Qt.ScrollBarPolicy.ScrollBarAlwaysOn);
        errors.setReadOnly(true);

        horizontalLayout_3.addWidget(errors);

        tabWidget.addTab(tab, new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/comment_error.png")), com.trolltech.qt.core.QCoreApplication.translate("Main", "Errors", null));
        tab_2 = new QWidget();
        tab_2.setObjectName("tab_2");
        horizontalLayout_4 = new QHBoxLayout(tab_2);
        horizontalLayout_4.setMargin(0);
        horizontalLayout_4.setObjectName("horizontalLayout_4");
        warnings = new QTextEdit(tab_2);
        warnings.setObjectName("warnings");
        warnings.setFrameShape(com.trolltech.qt.gui.QFrame.Shape.Box);
        warnings.setFrameShadow(com.trolltech.qt.gui.QFrame.Shadow.Plain);
        warnings.setLineWidth(0);
        warnings.setVerticalScrollBarPolicy(com.trolltech.qt.core.Qt.ScrollBarPolicy.ScrollBarAlwaysOn);
        warnings.setReadOnly(true);

        horizontalLayout_4.addWidget(warnings);

        tabWidget.addTab(tab_2, new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/comment_warning.png")), com.trolltech.qt.core.QCoreApplication.translate("Main", "Warnings", null));
        tab_3 = new QWidget();
        tab_3.setObjectName("tab_3");
        horizontalLayout_5 = new QHBoxLayout(tab_3);
        horizontalLayout_5.setMargin(0);
        horizontalLayout_5.setObjectName("horizontalLayout_5");
        traces = new QTextEdit(tab_3);
        traces.setObjectName("traces");
        traces.setFrameShape(com.trolltech.qt.gui.QFrame.Shape.Box);
        traces.setFrameShadow(com.trolltech.qt.gui.QFrame.Shadow.Plain);
        traces.setLineWidth(0);
        traces.setVerticalScrollBarPolicy(com.trolltech.qt.core.Qt.ScrollBarPolicy.ScrollBarAlwaysOn);
        traces.setReadOnly(true);

        horizontalLayout_5.addWidget(traces);

        tabWidget.addTab(tab_3, new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/comment_go.png")), com.trolltech.qt.core.QCoreApplication.translate("Main", "Trace", null));

        horizontalLayout_2.addWidget(tabWidget);

        dockWidget.setWidget(dockWidgetContents_2);
        Main.addDockWidget(com.trolltech.qt.core.Qt.DockWidgetArea.resolve(8), dockWidget);
        retranslateUi(Main);

        informationStack.setCurrentIndex(0);
        tabWidget.setCurrentIndex(0);


        Main.connectSlotsByName();
    } // setupUi

    void retranslateUi(QMainWindow Main)
    {
        Main.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("Main", "Cambridge AUV", null));
        actionSettings.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "Settings", null));
        actionMoo.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "Moo", null));
        actionPahh_You_wish.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "Pahh. You wish.", null));
        label_3.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/logo-small.png\" /></p></body></html>", null));
        address.setStatusTip("");
        connectButton.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "Connect", null));
        errorMessage.setText("");
        controlLED.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("Main", "Image Processing", null));
        controlLED.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/red-led.png\" /></p></body></html>", null));
        imageProcLED.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("Main", "Image Processing", null));
        imageProcLED.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/red-led.png\" /></p></body></html>", null));
        aiLED.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("Main", "Image Processing", null));
        aiLED.setText(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/red-led.png\" /></p></body></html>", null));
        backButton.setStyleSheet(com.trolltech.qt.core.QCoreApplication.translate("Main", "#backButton {\n"+
"	border-style: none;\n"+
"	background-color: rgb(213, 213, 213);\n"+
"}", null));
        backButton.setText("");
        dockWidget.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("Main", "Messages", null));
        all.setHtml(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">Cambridge AUV GUI</span></p></body></html>", null));
        tabWidget.setTabText(tabWidget.indexOf(tab_4), com.trolltech.qt.core.QCoreApplication.translate("Main", "All", null));
        errors.setHtml(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">Cambridge AUV GUI</span></p></body></html>", null));
        tabWidget.setTabText(tabWidget.indexOf(tab), com.trolltech.qt.core.QCoreApplication.translate("Main", "Errors", null));
        warnings.setHtml(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">Cambridge AUV GUI</span></p></body></html>", null));
        tabWidget.setTabText(tabWidget.indexOf(tab_2), com.trolltech.qt.core.QCoreApplication.translate("Main", "Warnings", null));
        traces.setHtml(com.trolltech.qt.core.QCoreApplication.translate("Main", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:8pt;\">Cambridge AUV GUI</span></p></body></html>", null));
        tabWidget.setTabText(tabWidget.indexOf(tab_3), com.trolltech.qt.core.QCoreApplication.translate("Main", "Trace", null));
    } // retranslateUi

}

