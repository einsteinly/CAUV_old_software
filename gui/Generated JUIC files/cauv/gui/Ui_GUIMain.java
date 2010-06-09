/********************************************************************************
** Form generated from reading ui file 'GUIMain.jui'
**
** Created: Wed 17. Mar 18:21:15 2010
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
    public QHBoxLayout horizontalLayout_4;
    public QHBoxLayout horizontalLayout_3;
    public QWidget widget_2;
    public QHBoxLayout horizontalLayout;
    public QScrollArea scrollArea;
    public QWidget scrollAreaWidgetContents_2;
    public QVBoxLayout verticalLayout_4;
    public QWidget widget_7;
    public QHBoxLayout horizontalLayout_2;
    public QSpacerItem horizontalSpacer_5;
    public QVBoxLayout iconLayout;
    public QSpacerItem horizontalSpacer_6;
    public QWidget widget;
    public QVBoxLayout verticalLayout_3;
    public QWidget widget_6;
    public QGridLayout gridLayout;
    public QLabel imageProcLED;
    public QLabel tartLED;
    public QLabel sonarLED;
    public QLabel label;
    public QLabel label_8;
    public QLabel label_9;
    public QSpacerItem horizontalSpacer_2;
    public QStackedWidget informationStack;
    public QWidget addressPage;
    public QGridLayout gridLayout_2;
    public QWidget page_2;
    public QWidget widget_4;
    public QVBoxLayout verticalLayout_2;
    public QPushButton controlsToggle;
    public QWidget controls;
    public QHBoxLayout horizontalLayout_5;
    public QSpacerItem horizontalSpacer_3;
    public QWidget widget_5;
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
        GUIMain.resize(new QSize(850, 550).expandedTo(GUIMain.minimumSizeHint()));
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
        horizontalLayout_4 = new QHBoxLayout(centralwidget);
        horizontalLayout_4.setSpacing(0);
        horizontalLayout_4.setMargin(0);
        horizontalLayout_4.setObjectName("horizontalLayout_4");
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3.setObjectName("horizontalLayout_3");
        widget_2 = new QWidget(centralwidget);
        widget_2.setObjectName("widget_2");
        widget_2.setMinimumSize(new QSize(150, 0));
        widget_2.setMaximumSize(new QSize(150, 16777215));
        QPalette palette1= new QPalette();
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(97, 97, 97));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(81, 81, 81));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(43, 43, 43));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(97, 97, 97));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(81, 81, 81));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(43, 43, 43));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(97, 97, 97));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(81, 81, 81));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(43, 43, 43));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(32, 32, 32));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(65, 65, 65));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        widget_2.setPalette(palette1);
        horizontalLayout = new QHBoxLayout(widget_2);
        horizontalLayout.setMargin(0);
        horizontalLayout.setObjectName("horizontalLayout");
        scrollArea = new QScrollArea(widget_2);
        scrollArea.setObjectName("scrollArea");
        scrollArea.setFrameShape(com.trolltech.qt.gui.QFrame.Shape.NoFrame);
        scrollArea.setLineWidth(0);
        scrollArea.setWidgetResizable(true);
        scrollAreaWidgetContents_2 = new QWidget();
        scrollAreaWidgetContents_2.setObjectName("scrollAreaWidgetContents_2");
        scrollAreaWidgetContents_2.setGeometry(new QRect(0, 0, 150, 548));
        verticalLayout_4 = new QVBoxLayout(scrollAreaWidgetContents_2);
        verticalLayout_4.setObjectName("verticalLayout_4");
        verticalLayout_4.setContentsMargins(0, 0, -1, 0);
        widget_7 = new QWidget(scrollAreaWidgetContents_2);
        widget_7.setObjectName("widget_7");
        QPalette palette2= new QPalette();
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette2.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette2.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(16, 16, 16));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(33, 33, 33));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette2.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        widget_7.setPalette(palette2);
        widget_7.setAutoFillBackground(true);
        horizontalLayout_2 = new QHBoxLayout(widget_7);
        horizontalLayout_2.setObjectName("horizontalLayout_2");
        horizontalSpacer_5 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_2.addItem(horizontalSpacer_5);

        iconLayout = new QVBoxLayout();
        iconLayout.setObjectName("iconLayout");

        horizontalLayout_2.addLayout(iconLayout);

        horizontalSpacer_6 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_2.addItem(horizontalSpacer_6);


        verticalLayout_4.addWidget(widget_7);

        scrollArea.setWidget(scrollAreaWidgetContents_2);

        horizontalLayout.addWidget(scrollArea);


        horizontalLayout_3.addWidget(widget_2);

        widget = new QWidget(centralwidget);
        widget.setObjectName("widget");
        verticalLayout_3 = new QVBoxLayout(widget);
        verticalLayout_3.setSpacing(0);
        verticalLayout_3.setMargin(0);
        verticalLayout_3.setObjectName("verticalLayout_3");
        widget_6 = new QWidget(widget);
        widget_6.setObjectName("widget_6");
        widget_6.setAutoFillBackground(true);
        gridLayout = new QGridLayout(widget_6);
        gridLayout.setMargin(0);
        gridLayout.setObjectName("gridLayout");
        imageProcLED = new QLabel(widget_6);
        imageProcLED.setObjectName("imageProcLED");
        imageProcLED.setMinimumSize(new QSize(20, 20));
        imageProcLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignLeft,com.trolltech.qt.core.Qt.AlignmentFlag.AlignTop));

        gridLayout.addWidget(imageProcLED, 0, 2, 1, 1);

        tartLED = new QLabel(widget_6);
        tartLED.setObjectName("tartLED");
        tartLED.setMinimumSize(new QSize(20, 20));
        tartLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignLeft,com.trolltech.qt.core.Qt.AlignmentFlag.AlignTop));

        gridLayout.addWidget(tartLED, 0, 4, 1, 1);

        sonarLED = new QLabel(widget_6);
        sonarLED.setObjectName("sonarLED");
        sonarLED.setMinimumSize(new QSize(20, 20));
        sonarLED.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignLeft,com.trolltech.qt.core.Qt.AlignmentFlag.AlignTop));

        gridLayout.addWidget(sonarLED, 0, 6, 1, 1);

        label = new QLabel(widget_6);
        label.setObjectName("label");
        label.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        gridLayout.addWidget(label, 0, 1, 1, 1);

        label_8 = new QLabel(widget_6);
        label_8.setObjectName("label_8");
        label_8.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        gridLayout.addWidget(label_8, 0, 3, 1, 1);

        label_9 = new QLabel(widget_6);
        label_9.setObjectName("label_9");
        label_9.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        gridLayout.addWidget(label_9, 0, 5, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout.addItem(horizontalSpacer_2, 0, 0, 1, 1);


        verticalLayout_3.addWidget(widget_6);

        informationStack = new QStackedWidget(widget);
        informationStack.setObjectName("informationStack");
        addressPage = new QWidget();
        addressPage.setObjectName("addressPage");
        gridLayout_2 = new QGridLayout(addressPage);
        gridLayout_2.setObjectName("gridLayout_2");
        informationStack.addWidget(addressPage);
        page_2 = new QWidget();
        page_2.setObjectName("page_2");
        informationStack.addWidget(page_2);

        verticalLayout_3.addWidget(informationStack);

        widget_4 = new QWidget(widget);
        widget_4.setObjectName("widget_4");
        widget_4.setMaximumSize(new QSize(16777215, 250));
        QPalette palette3= new QPalette();
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette3.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette3.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(16, 16, 16));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(33, 33, 33));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette3.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        widget_4.setPalette(palette3);
        verticalLayout_2 = new QVBoxLayout(widget_4);
        verticalLayout_2.setSpacing(0);
        verticalLayout_2.setMargin(0);
        verticalLayout_2.setObjectName("verticalLayout_2");
        controlsToggle = new QPushButton(widget_4);
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

        controls = new QWidget(widget_4);
        controls.setObjectName("controls");
        controls.setMinimumSize(new QSize(0, 130));
        controls.setMaximumSize(new QSize(16777215, 130));
        controls.setAutoFillBackground(true);
        horizontalLayout_5 = new QHBoxLayout(controls);
        horizontalLayout_5.setSpacing(0);
        horizontalLayout_5.setMargin(0);
        horizontalLayout_5.setObjectName("horizontalLayout_5");
        horizontalSpacer_3 = new QSpacerItem(28, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_5.addItem(horizontalSpacer_3);

        widget_5 = new QWidget(controls);
        widget_5.setObjectName("widget_5");
        widget_5.setMinimumSize(new QSize(611, 130));
        widget_5.setAutoFillBackground(false);
        shutdownButton = new QPushButton(widget_5);
        shutdownButton.setObjectName("shutdownButton");
        shutdownButton.setGeometry(new QRect(40, 20, 101, 91));
        QPalette palette5= new QPalette();
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(204, 141, 144));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette5.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(204, 141, 144));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette5.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(77, 14, 17));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(77, 14, 17));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(77, 14, 17));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(154, 28, 34));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette5.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        shutdownButton.setPalette(palette5);
        shutdownButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/red_button.png")));
        shutdownButton.setIconSize(new QSize(120, 120));
        shutdownButton.setFlat(true);
        sliderProp = new QSlider(widget_5);
        sliderProp.setObjectName("sliderProp");
        sliderProp.setGeometry(new QRect(190, 10, 20, 81));
        sliderProp.setMinimum(-100);
        sliderProp.setMaximum(100);
        sliderProp.setOrientation(com.trolltech.qt.core.Qt.Orientation.Vertical);
        sliderHBow = new QSlider(widget_5);
        sliderHBow.setObjectName("sliderHBow");
        sliderHBow.setGeometry(new QRect(300, 30, 151, 19));
        sliderHBow.setMinimum(-100);
        sliderHBow.setMaximum(100);
        sliderHBow.setOrientation(com.trolltech.qt.core.Qt.Orientation.Horizontal);
        sliderVBow = new QSlider(widget_5);
        sliderVBow.setObjectName("sliderVBow");
        sliderVBow.setGeometry(new QRect(500, 10, 20, 81));
        sliderVBow.setMinimum(-100);
        sliderVBow.setMaximum(100);
        sliderVBow.setOrientation(com.trolltech.qt.core.Qt.Orientation.Vertical);
        sliderVStern = new QSlider(widget_5);
        sliderVStern.setObjectName("sliderVStern");
        sliderVStern.setGeometry(new QRect(550, 10, 20, 81));
        sliderVStern.setMinimum(-100);
        sliderVStern.setMaximum(100);
        sliderVStern.setOrientation(com.trolltech.qt.core.Qt.Orientation.Vertical);
        sliderHStern = new QSlider(widget_5);
        sliderHStern.setObjectName("sliderHStern");
        sliderHStern.setGeometry(new QRect(300, 80, 151, 19));
        sliderHStern.setMinimum(-100);
        sliderHStern.setMaximum(100);
        sliderHStern.setOrientation(com.trolltech.qt.core.Qt.Orientation.Horizontal);
        label_3 = new QLabel(widget_5);
        label_3.setObjectName("label_3");
        label_3.setGeometry(new QRect(180, 100, 41, 16));
        label_3.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));
        label_4 = new QLabel(widget_5);
        label_4.setObjectName("label_4");
        label_4.setGeometry(new QRect(260, 30, 31, 20));
        label_4.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));
        label_5 = new QLabel(widget_5);
        label_5.setObjectName("label_5");
        label_5.setGeometry(new QRect(250, 80, 41, 20));
        label_5.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));
        label_6 = new QLabel(widget_5);
        label_6.setObjectName("label_6");
        label_6.setGeometry(new QRect(490, 100, 41, 16));
        label_6.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));
        label_7 = new QLabel(widget_5);
        label_7.setObjectName("label_7");
        label_7.setGeometry(new QRect(540, 100, 41, 16));
        label_7.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        horizontalLayout_5.addWidget(widget_5);

        horizontalSpacer_4 = new QSpacerItem(29, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_5.addItem(horizontalSpacer_4);


        verticalLayout_2.addWidget(controls);


        verticalLayout_3.addWidget(widget_4);


        horizontalLayout_3.addWidget(widget);


        horizontalLayout_4.addLayout(horizontalLayout_3);

        GUIMain.setCentralWidget(centralwidget);
        retranslateUi(GUIMain);

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
        label.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Image", null));
        label_8.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "TART", null));
        label_9.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Sonar", null));
        controlsToggle.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Control Mode", null));
        shutdownButton.setText("");
        label_3.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "Prop", null));
        label_4.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "H Bow", null));
        label_5.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "H Stern", null));
        label_6.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "V Bow", null));
        label_7.setText(com.trolltech.qt.core.QCoreApplication.translate("GUIMain", "V Stern", null));
    } // retranslateUi

}

