/********************************************************************************
** Form generated from reading ui file 'GUIMain.jui'
**
** Created: Tue 22. Jun 17:01:06 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_GUIMain implements com.trolltech.qt.QUiForm<QMainWindow>
{
    public QWidget centralwidget;
    public QVBoxLayout verticalLayout;
    public QWidget widget_3;
    public QHBoxLayout horizontalLayout_4;
    public QHBoxLayout iconLayout;
    public QSpacerItem horizontalSpacer_7;
    public QWidget statusLights;
    public QGridLayout gridLayout;
    public QSpacerItem verticalSpacer;
    public QLabel imageProcLED;
    public QLabel tartLED;
    public QLabel sonarLED;
    public QWidget widget;
    public QHBoxLayout horizontalLayout_3;
    public QStackedWidget mainStack;
    public QWidget addressPage;
    public QHBoxLayout horizontalLayout;
    public QWidget addressContainer;
    public QHBoxLayout horizontalLayout_2;
    public QSpacerItem horizontalSpacer;
    public QWidget widget_2;
    public QCommandLinkButton connectButton;
    public QLabel auvAddressLabel_3;
    public QLabel auvAddressLabel_4;
    public QLabel label_2;
    public QSpinBox port;
    public QLineEdit address;
    public QLabel errorMessage;
    public QSpacerItem horizontalSpacer_2;
    public QWidget informationPage;
    public QVBoxLayout verticalLayout_2;
    public QStackedWidget informationStack;
    public QWidget iconsPage;
    public QWidget page_2;
    public QPushButton controlsToggle;
    public QWidget controls;
    public QHBoxLayout horizontalLayout_5;
    public QSpacerItem horizontalSpacer_3;
    public QWidget widget_4;
    public QPushButton shutdownButton;
    public QSlider sliderProp;
    public QSlider sliderHBow;
    public QSlider sliderVBow;
    public QSlider sliderVStern;
    public QSlider sliderHStern;
    public QLabel label_3;
    public QLabel label_4;
    public QLabel label_5;
    public QLabel label_6;
    public QLabel label_7;
    public QSpacerItem horizontalSpacer_4;

    public Ui_GUIMain() { super(); }

    public void setupUi(QMainWindow GUIMain)
    {
        GUIMain.setObjectName("GUIMain");
        GUIMain.resize(new QSize(747, 489).expandedTo(GUIMain.minimumSizeHint()));
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
        GUIMain.setPalette(palette);
        GUIMain.setWindowIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icon.png")));
        centralwidget = new QWidget(GUIMain);
        centralwidget.setObjectName("centralwidget");
        centralwidget.setEnabled(true);
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout.setSpacing(0);
        verticalLayout.setMargin(0);
        verticalLayout.setObjectName("verticalLayout");
        widget_3 = new QWidget(centralwidget);
        widget_3.setObjectName("widget_3");
        widget_3.setMinimumSize(new QSize(686, 120));
        widget_3.setMaximumSize(new QSize(16777215, 120));
        QPalette palette1= new QPalette();
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(40, 40, 40));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(33, 33, 33));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(18, 18, 18));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(40, 40, 40));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(33, 33, 33));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(18, 18, 18));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(40, 40, 40));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(33, 33, 33));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(18, 18, 18));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(13, 13, 13));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(27, 27, 27));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        widget_3.setPalette(palette1);
        widget_3.setAutoFillBackground(true);
        horizontalLayout_4 = new QHBoxLayout(widget_3);
        horizontalLayout_4.setSpacing(0);
        horizontalLayout_4.setMargin(0);
        horizontalLayout_4.setObjectName("horizontalLayout_4");
        iconLayout = new QHBoxLayout();
        iconLayout.setSpacing(0);
        iconLayout.setObjectName("iconLayout");
        iconLayout.setContentsMargins(9, -1, -1, -1);

        horizontalLayout_4.addLayout(iconLayout);

        horizontalSpacer_7 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_4.addItem(horizontalSpacer_7);

        statusLights = new QWidget(widget_3);
        statusLights.setObjectName("statusLights");
        gridLayout = new QGridLayout(statusLights);
        gridLayout.setObjectName("gridLayout");
        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout.addItem(verticalSpacer, 4, 1, 1, 1);

        imageProcLED = new QLabel(statusLights);
        imageProcLED.setObjectName("imageProcLED");
        imageProcLED.setMinimumSize(new QSize(20, 20));
        imageProcLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignLeft,com.trolltech.qt.core.Qt.AlignmentFlag.AlignTop));

        gridLayout.addWidget(imageProcLED, 0, 4, 1, 1);

        tartLED = new QLabel(statusLights);
        tartLED.setObjectName("tartLED");
        tartLED.setMinimumSize(new QSize(20, 20));
        tartLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignLeft,com.trolltech.qt.core.Qt.AlignmentFlag.AlignTop));

        gridLayout.addWidget(tartLED, 1, 4, 1, 1);

        sonarLED = new QLabel(statusLights);
        sonarLED.setObjectName("sonarLED");
        sonarLED.setMinimumSize(new QSize(20, 20));
        sonarLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignLeft,com.trolltech.qt.core.Qt.AlignmentFlag.AlignTop));

        gridLayout.addWidget(sonarLED, 2, 4, 1, 1);


        horizontalLayout_4.addWidget(statusLights);


        verticalLayout.addWidget(widget_3);

        widget = new QWidget(centralwidget);
        widget.setObjectName("widget");
        horizontalLayout_3 = new QHBoxLayout(widget);
        horizontalLayout_3.setMargin(0);
        horizontalLayout_3.setObjectName("horizontalLayout_3");
        mainStack = new QStackedWidget(widget);
        mainStack.setObjectName("mainStack");
        mainStack.setAutoFillBackground(false);
        addressPage = new QWidget();
        addressPage.setObjectName("addressPage");
        horizontalLayout = new QHBoxLayout(addressPage);
        horizontalLayout.setObjectName("horizontalLayout");
        addressContainer = new QWidget(addressPage);
        addressContainer.setObjectName("addressContainer");
        horizontalLayout_2 = new QHBoxLayout(addressContainer);
        horizontalLayout_2.setObjectName("horizontalLayout_2");
        horizontalSpacer = new QSpacerItem(91, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_2.addItem(horizontalSpacer);

        widget_2 = new QWidget(addressContainer);
        widget_2.setObjectName("widget_2");
        widget_2.setMinimumSize(new QSize(451, 300));
        widget_2.setMaximumSize(new QSize(451, 300));
        connectButton = new QCommandLinkButton(widget_2);
        connectButton.setObjectName("connectButton");
        connectButton.setGeometry(new QRect(250, 170, 111, 41));
        QFont font = new QFont();
        connectButton.setFont(font);
        connectButton.setStyleSheet("#connectButton{\n"+
"	font-size: 18px;\n"+
"	color: rgb(255, 255, 255);\n"+
"}");
        connectButton.setIconSize(new QSize(20, 20));
        auvAddressLabel_3 = new QLabel(widget_2);
        auvAddressLabel_3.setObjectName("auvAddressLabel_3");
        auvAddressLabel_3.setGeometry(new QRect(45, 170, 81, 21));
        auvAddressLabel_4 = new QLabel(widget_2);
        auvAddressLabel_4.setObjectName("auvAddressLabel_4");
        auvAddressLabel_4.setGeometry(new QRect(25, 130, 101, 31));
        label_2 = new QLabel(widget_2);
        label_2.setObjectName("label_2");
        label_2.setGeometry(new QRect(130, 20, 241, 81));
        port = new QSpinBox(widget_2);
        port.setObjectName("port");
        port.setEnabled(true);
        port.setGeometry(new QRect(140, 170, 91, 21));
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
        port.setMinimum(-1);
        port.setMaximum(999999);
        address = new QLineEdit(widget_2);
        address.setObjectName("address");
        address.setEnabled(true);
        address.setGeometry(new QRect(140, 130, 221, 31));
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
        address.setPalette(palette3);
        address.setLayoutDirection(com.trolltech.qt.core.Qt.LayoutDirection.LeftToRight);
        errorMessage = new QLabel(widget_2);
        errorMessage.setObjectName("errorMessage");
        errorMessage.setEnabled(true);
        errorMessage.setGeometry(new QRect(140, 219, 221, 41));
        errorMessage.setStyleSheet("#errorMessage{\n"+
"	font-size: 12px;\n"+
"	color: rgb(255, 0, 0);\n"+
"	font-weight: bold;\n"+
"	text-align: center;\n"+
"}");
        errorMessage.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));
        errorMessage.setWordWrap(true);

        horizontalLayout_2.addWidget(widget_2);

        horizontalSpacer_2 = new QSpacerItem(90, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_2.addItem(horizontalSpacer_2);


        horizontalLayout.addWidget(addressContainer);

        mainStack.addWidget(addressPage);
        informationPage = new QWidget();
        informationPage.setObjectName("informationPage");
        verticalLayout_2 = new QVBoxLayout(informationPage);
        verticalLayout_2.setSpacing(0);
        verticalLayout_2.setMargin(0);
        verticalLayout_2.setObjectName("verticalLayout_2");
        informationStack = new QStackedWidget(informationPage);
        informationStack.setObjectName("informationStack");
        iconsPage = new QWidget();
        iconsPage.setObjectName("iconsPage");
        informationStack.addWidget(iconsPage);
        page_2 = new QWidget();
        page_2.setObjectName("page_2");
        informationStack.addWidget(page_2);

        verticalLayout_2.addWidget(informationStack);

        controlsToggle = new QPushButton(informationPage);
        controlsToggle.setObjectName("controlsToggle");
        QPalette palette4= new QPalette();
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(97, 97, 97));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(81, 81, 81));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(43, 43, 43));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette4.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(97, 97, 97));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(81, 81, 81));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(43, 43, 43));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette4.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(97, 97, 97));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(81, 81, 81));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(43, 43, 43));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(32, 32, 32));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(65, 65, 65));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette4.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        controlsToggle.setPalette(palette4);
        controlsToggle.setFocusPolicy(com.trolltech.qt.core.Qt.FocusPolicy.NoFocus);
        controlsToggle.setAutoFillBackground(true);
        controlsToggle.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/arrow.png")));
        controlsToggle.setCheckable(false);
        controlsToggle.setChecked(false);
        controlsToggle.setFlat(true);

        verticalLayout_2.addWidget(controlsToggle);

        controls = new QWidget(informationPage);
        controls.setObjectName("controls");
        controls.setMinimumSize(new QSize(0, 130));
        controls.setMaximumSize(new QSize(16777215, 130));
        QPalette palette5= new QPalette();
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(60, 60, 60));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(50, 50, 50));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(26, 26, 26));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(60, 60, 60));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(50, 50, 50));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(26, 26, 26));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(60, 60, 60));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(50, 50, 50));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(26, 26, 26));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(20, 20, 20));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(40, 40, 40));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        controls.setPalette(palette5);
        controls.setAutoFillBackground(true);
        horizontalLayout_5 = new QHBoxLayout(controls);
        horizontalLayout_5.setMargin(0);
        horizontalLayout_5.setObjectName("horizontalLayout_5");
        horizontalSpacer_3 = new QSpacerItem(28, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_5.addItem(horizontalSpacer_3);

        widget_4 = new QWidget(controls);
        widget_4.setObjectName("widget_4");
        widget_4.setMinimumSize(new QSize(611, 130));
        shutdownButton = new QPushButton(widget_4);
        shutdownButton.setObjectName("shutdownButton");
        shutdownButton.setGeometry(new QRect(40, 20, 101, 91));
        QPalette palette6= new QPalette();
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(204, 141, 144));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette6.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(204, 141, 144));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette6.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(77, 14, 17));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(77, 14, 17));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(77, 14, 17));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(154, 28, 34));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette6.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        shutdownButton.setPalette(palette6);
        shutdownButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/red_button.png")));
        shutdownButton.setIconSize(new QSize(120, 120));
        shutdownButton.setFlat(true);
        sliderProp = new QSlider(widget_4);
        sliderProp.setObjectName("sliderProp");
        sliderProp.setGeometry(new QRect(190, 10, 20, 81));
        sliderProp.setMinimum(-100);
        sliderProp.setMaximum(100);
        sliderProp.setOrientation(com.trolltech.qt.core.Qt.Orientation.Vertical);
        sliderHBow = new QSlider(widget_4);
        sliderHBow.setObjectName("sliderHBow");
        sliderHBow.setGeometry(new QRect(300, 30, 151, 19));
        sliderHBow.setMinimum(-100);
        sliderHBow.setMaximum(100);
        sliderHBow.setOrientation(com.trolltech.qt.core.Qt.Orientation.Horizontal);
        sliderVBow = new QSlider(widget_4);
        sliderVBow.setObjectName("sliderVBow");
        sliderVBow.setGeometry(new QRect(500, 10, 20, 81));
        sliderVBow.setMinimum(-100);
        sliderVBow.setMaximum(100);
        sliderVBow.setOrientation(com.trolltech.qt.core.Qt.Orientation.Vertical);
        sliderVStern = new QSlider(widget_4);
        sliderVStern.setObjectName("sliderVStern");
        sliderVStern.setGeometry(new QRect(550, 10, 20, 81));
        sliderVStern.setMinimum(-100);
        sliderVStern.setMaximum(100);
        sliderVStern.setOrientation(com.trolltech.qt.core.Qt.Orientation.Vertical);
        sliderHStern = new QSlider(widget_4);
        sliderHStern.setObjectName("sliderHStern");
        sliderHStern.setGeometry(new QRect(300, 80, 151, 19));
        sliderHStern.setMinimum(-100);
        sliderHStern.setMaximum(100);
        sliderHStern.setOrientation(com.trolltech.qt.core.Qt.Orientation.Horizontal);
        label_3 = new QLabel(widget_4);
        label_3.setObjectName("label_3");
        label_3.setGeometry(new QRect(180, 100, 41, 16));
        label_3.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));
        label_4 = new QLabel(widget_4);
        label_4.setObjectName("label_4");
        label_4.setGeometry(new QRect(260, 30, 31, 20));
        label_4.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));
        label_5 = new QLabel(widget_4);
        label_5.setObjectName("label_5");
        label_5.setGeometry(new QRect(250, 80, 41, 20));
        label_5.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));
        label_6 = new QLabel(widget_4);
        label_6.setObjectName("label_6");
        label_6.setGeometry(new QRect(490, 100, 41, 16));
        label_6.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));
        label_7 = new QLabel(widget_4);
        label_7.setObjectName("label_7");
        label_7.setGeometry(new QRect(540, 100, 41, 16));
        label_7.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        horizontalLayout_5.addWidget(widget_4);

        horizontalSpacer_4 = new QSpacerItem(29, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_5.addItem(horizontalSpacer_4);


        verticalLayout_2.addWidget(controls);

        mainStack.addWidget(informationPage);

        horizontalLayout_3.addWidget(mainStack);


        verticalLayout.addWidget(widget);

        GUIMain.setCentralWidget(centralwidget);
        auvAddressLabel_3.setBuddy(port);
        auvAddressLabel_4.setBuddy(address);
        retranslateUi(GUIMain);

        mainStack.setCurrentIndex(1);


        GUIMain.connectSlotsByName();
    } // setupUi

    void retranslateUi(QMainWindow GUIMain)
    {
        GUIMain.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Cambridge AUV", null));
        GUIMain.setStyleSheet("");
        imageProcLED.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Image Processing", null));
        imageProcLED.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/red-led.png\" /></p></body></html>", null));
        tartLED.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "TART", null));
        tartLED.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/red-led.png\" /></p></body></html>", null));
        sonarLED.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Sonar", null));
        sonarLED.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/red-led.png\" /></p></body></html>", null));
        mainStack.setStyleSheet("");
        connectButton.setStatusTip("");
        connectButton.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Connect", null));
        connectButton.setDescription("");
        auvAddressLabel_3.setStyleSheet("");
        auvAddressLabel_3.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p align=\"right\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Euphemia'; font-size:10pt; font-weight:600; color:#ececec;\">Port:</span></p></body></html>", null));
        auvAddressLabel_4.setStyleSheet("");
        auvAddressLabel_4.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p align=\"right\" style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-family:'Euphemia'; font-size:12pt; font-weight:600; color:#ffffff;\">Address:</span></p></body></html>", null));
        label_2.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"+
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"+
"p, li { white-space: pre-wrap; }\n"+
"</style></head><body style=\" font-family:'MS Shell Dlg 2'; font-size:8.25pt; font-weight:400; font-style:normal;\">\n"+
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><img src=\"classpath:cauv/gui/resources/logo.png\" /></p></body></html>", null));
        errorMessage.setText("");
        controlsToggle.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Control Mode", null));
        shutdownButton.setText("");
        label_3.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Prop", null));
        label_4.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "H Bow", null));
        label_5.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "H Stern", null));
        label_6.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "V Bow", null));
        label_7.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "V Stern", null));
    } // retranslateUi

}

