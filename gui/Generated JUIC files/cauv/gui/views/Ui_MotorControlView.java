/********************************************************************************
** Form generated from reading ui file 'MotorControlView.jui'
**
** Created: Thu 6. May 01:36:59 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui.views;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_MotorControlView implements com.trolltech.qt.QUiForm<QWidget>
{
    public QGridLayout gridLayout;
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
    public QSpacerItem verticalSpacer;
    public QSpacerItem verticalSpacer_2;
    public QSpacerItem horizontalSpacer;
    public QSpacerItem horizontalSpacer_2;

    public Ui_MotorControlView() { super(); }

    public void setupUi(QWidget MotorControlView)
    {
        MotorControlView.setObjectName("MotorControlView");
        MotorControlView.resize(new QSize(691, 274).expandedTo(MotorControlView.minimumSizeHint()));
        gridLayout = new QGridLayout(MotorControlView);
        gridLayout.setObjectName("gridLayout");
        widget_4 = new QWidget(MotorControlView);
        widget_4.setObjectName("widget_4");
        widget_4.setMinimumSize(new QSize(611, 130));
        shutdownButton = new QPushButton(widget_4);
        shutdownButton.setObjectName("shutdownButton");
        shutdownButton.setGeometry(new QRect(40, 20, 101, 91));
        QPalette palette= new QPalette();
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(204, 141, 144));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(204, 141, 144));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(77, 14, 17));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(231, 42, 51));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(192, 35, 42));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(77, 14, 17));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(103, 18, 22));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(77, 14, 17));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(77, 14, 17));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(154, 28, 34));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        shutdownButton.setPalette(palette);
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

        gridLayout.addWidget(widget_4, 1, 1, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout.addItem(verticalSpacer, 0, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout.addItem(verticalSpacer_2, 2, 1, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout.addItem(horizontalSpacer, 1, 2, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout.addItem(horizontalSpacer_2, 1, 0, 1, 1);

        retranslateUi(MotorControlView);

        MotorControlView.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget MotorControlView)
    {
        MotorControlView.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("MotorControlView", "Form", null));
        shutdownButton.setText("");
        label_3.setText(com.trolltech.qt.core.QCoreApplication.translate("MotorControlView", "Prop", null));
        label_4.setText(com.trolltech.qt.core.QCoreApplication.translate("MotorControlView", "H Bow", null));
        label_5.setText(com.trolltech.qt.core.QCoreApplication.translate("MotorControlView", "H Stern", null));
        label_6.setText(com.trolltech.qt.core.QCoreApplication.translate("MotorControlView", "V Bow", null));
        label_7.setText(com.trolltech.qt.core.QCoreApplication.translate("MotorControlView", "V Stern", null));
    } // retranslateUi

}

