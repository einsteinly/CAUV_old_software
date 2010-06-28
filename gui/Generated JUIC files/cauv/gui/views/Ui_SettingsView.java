/********************************************************************************
** Form generated from reading ui file 'SettingsView.jui'
**
** Created: Fri 25. Jun 13:17:48 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui.views;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_SettingsView implements com.trolltech.qt.QUiForm<QWidget>
{
    public QGridLayout gridLayout_2;
    public QToolBox toolBox;
    public QWidget page_2;
    public QGridLayout gridLayout_3;
    public QSpacerItem horizontalSpacer;
    public QWidget widget;
    public QGridLayout gridLayout;
    public QDoubleSpinBox yaw_Ki;
    public QDoubleSpinBox yaw_Kd;
    public QDoubleSpinBox yaw_scale;
    public QDoubleSpinBox pitch_Ki;
    public QDoubleSpinBox pitch_Kd;
    public QDoubleSpinBox pitch_scale;
    public QDoubleSpinBox depth_Ki;
    public QDoubleSpinBox depth_Kd;
    public QDoubleSpinBox depth_scale;
    public QLabel label_3;
    public QLabel label_4;
    public QLabel label_5;
    public QLabel label_6;
    public QDoubleSpinBox yaw_Kp;
    public QDoubleSpinBox pitch_Kp;
    public QDoubleSpinBox depth_Kp;
    public QLabel label_2;
    public QLabel label_7;
    public QLabel label_8;
    public QSpacerItem horizontalSpacer_2;
    public QSpacerItem verticalSpacer;
    public QSpacerItem verticalSpacer_2;
    public QWidget page;
    public QHBoxLayout horizontalLayout;
    public QSpacerItem horizontalSpacer_3;
    public QWidget widget_2;
    public QLabel label;
    public QSpinBox gamepadID;
    public QPushButton saveSettingsButton;
    public QSpacerItem horizontalSpacer_4;

    public Ui_SettingsView() { super(); }

    public void setupUi(QWidget SettingsView)
    {
        SettingsView.setObjectName("SettingsView");
        SettingsView.resize(new QSize(568, 354).expandedTo(SettingsView.minimumSizeHint()));
        gridLayout_2 = new QGridLayout(SettingsView);
        gridLayout_2.setObjectName("gridLayout_2");
        toolBox = new QToolBox(SettingsView);
        toolBox.setObjectName("toolBox");
        page_2 = new QWidget();
        page_2.setObjectName("page_2");
        page_2.setGeometry(new QRect(0, 0, 550, 280));
        gridLayout_3 = new QGridLayout(page_2);
        gridLayout_3.setObjectName("gridLayout_3");
        horizontalSpacer = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout_3.addItem(horizontalSpacer, 1, 0, 1, 1);

        widget = new QWidget(page_2);
        widget.setObjectName("widget");
        gridLayout = new QGridLayout(widget);
        gridLayout.setObjectName("gridLayout");
        yaw_Ki = new QDoubleSpinBox(widget);
        yaw_Ki.setObjectName("yaw_Ki");

        gridLayout.addWidget(yaw_Ki, 2, 3, 1, 1);

        yaw_Kd = new QDoubleSpinBox(widget);
        yaw_Kd.setObjectName("yaw_Kd");

        gridLayout.addWidget(yaw_Kd, 2, 4, 1, 1);

        yaw_scale = new QDoubleSpinBox(widget);
        yaw_scale.setObjectName("yaw_scale");

        gridLayout.addWidget(yaw_scale, 2, 5, 1, 1);

        pitch_Ki = new QDoubleSpinBox(widget);
        pitch_Ki.setObjectName("pitch_Ki");

        gridLayout.addWidget(pitch_Ki, 6, 3, 1, 1);

        pitch_Kd = new QDoubleSpinBox(widget);
        pitch_Kd.setObjectName("pitch_Kd");

        gridLayout.addWidget(pitch_Kd, 6, 4, 1, 1);

        pitch_scale = new QDoubleSpinBox(widget);
        pitch_scale.setObjectName("pitch_scale");

        gridLayout.addWidget(pitch_scale, 6, 5, 1, 1);

        depth_Ki = new QDoubleSpinBox(widget);
        depth_Ki.setObjectName("depth_Ki");

        gridLayout.addWidget(depth_Ki, 7, 3, 1, 1);

        depth_Kd = new QDoubleSpinBox(widget);
        depth_Kd.setObjectName("depth_Kd");

        gridLayout.addWidget(depth_Kd, 7, 4, 1, 1);

        depth_scale = new QDoubleSpinBox(widget);
        depth_scale.setObjectName("depth_scale");

        gridLayout.addWidget(depth_scale, 7, 5, 1, 1);

        label_3 = new QLabel(widget);
        label_3.setObjectName("label_3");
        label_3.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_3, 1, 3, 1, 1);

        label_4 = new QLabel(widget);
        label_4.setObjectName("label_4");
        label_4.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_4, 1, 4, 1, 1);

        label_5 = new QLabel(widget);
        label_5.setObjectName("label_5");
        label_5.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_5, 1, 5, 1, 1);

        label_6 = new QLabel(widget);
        label_6.setObjectName("label_6");
        label_6.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_6, 2, 1, 1, 1);

        yaw_Kp = new QDoubleSpinBox(widget);
        yaw_Kp.setObjectName("yaw_Kp");

        gridLayout.addWidget(yaw_Kp, 2, 2, 1, 1);

        pitch_Kp = new QDoubleSpinBox(widget);
        pitch_Kp.setObjectName("pitch_Kp");

        gridLayout.addWidget(pitch_Kp, 6, 2, 1, 1);

        depth_Kp = new QDoubleSpinBox(widget);
        depth_Kp.setObjectName("depth_Kp");

        gridLayout.addWidget(depth_Kp, 7, 2, 1, 1);

        label_2 = new QLabel(widget);
        label_2.setObjectName("label_2");
        label_2.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_2, 1, 2, 1, 1);

        label_7 = new QLabel(widget);
        label_7.setObjectName("label_7");
        label_7.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_7, 6, 1, 1, 1);

        label_8 = new QLabel(widget);
        label_8.setObjectName("label_8");
        label_8.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_8, 7, 1, 1, 1);


        gridLayout_3.addWidget(widget, 1, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout_3.addItem(horizontalSpacer_2, 1, 2, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout_3.addItem(verticalSpacer, 0, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout_3.addItem(verticalSpacer_2, 2, 1, 1, 1);

        toolBox.addItem(page_2, com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Autopilot Settings", null));
        page = new QWidget();
        page.setObjectName("page");
        page.setGeometry(new QRect(0, 0, 550, 280));
        horizontalLayout = new QHBoxLayout(page);
        horizontalLayout.setObjectName("horizontalLayout");
        horizontalSpacer_3 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout.addItem(horizontalSpacer_3);

        widget_2 = new QWidget(page);
        widget_2.setObjectName("widget_2");
        widget_2.setMinimumSize(new QSize(200, 0));
        label = new QLabel(widget_2);
        label.setObjectName("label");
        label.setGeometry(new QRect(20, 20, 59, 21));
        gamepadID = new QSpinBox(widget_2);
        gamepadID.setObjectName("gamepadID");
        gamepadID.setGeometry(new QRect(90, 20, 61, 20));
        QPalette palette= new QPalette();
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(127, 127, 127));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(127, 127, 127));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(170, 170, 170));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(127, 127, 127));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(127, 127, 127));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        gamepadID.setPalette(palette);
        saveSettingsButton = new QPushButton(widget_2);
        saveSettingsButton.setObjectName("saveSettingsButton");
        saveSettingsButton.setGeometry(new QRect(80, 50, 71, 23));
        QPalette palette1= new QPalette();
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(243, 243, 243));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(116, 116, 116));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(155, 155, 155));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(243, 243, 243));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(243, 243, 243));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(116, 116, 116));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(155, 155, 155));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(243, 243, 243));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(116, 116, 116));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(243, 243, 243));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(116, 116, 116));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(155, 155, 155));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(116, 116, 116));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(116, 116, 116));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(232, 232, 232));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette1.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        saveSettingsButton.setPalette(palette1);

        horizontalLayout.addWidget(widget_2);

        horizontalSpacer_4 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout.addItem(horizontalSpacer_4);

        toolBox.addItem(page, com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Gamepad Settings", null));

        gridLayout_2.addWidget(toolBox, 0, 0, 1, 1);

        retranslateUi(SettingsView);

        toolBox.setCurrentIndex(0);


        SettingsView.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget SettingsView)
    {
        SettingsView.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Form", null));
        label_3.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Ki", null));
        label_4.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Kd", null));
        label_5.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "scale", null));
        label_6.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Yaw", null));
        label_2.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Kp", null));
        label_7.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Pitch", null));
        label_8.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Depth", null));
        toolBox.setItemText(toolBox.indexOf(page_2), com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Autopilot Settings", null));
        label.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Gamepad ID", null));
        saveSettingsButton.setText(com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Save", null));
        toolBox.setItemText(toolBox.indexOf(page), com.trolltech.qt.core.QCoreApplication.translate("SettingsView", "Gamepad Settings", null));
    } // retranslateUi

}

