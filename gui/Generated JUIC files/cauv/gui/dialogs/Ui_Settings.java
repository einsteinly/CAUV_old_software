/********************************************************************************
** Form generated from reading ui file 'Settings.jui'
**
** Created: Sat 3. Jul 08:41:13 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui.dialogs;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_Settings implements com.trolltech.qt.QUiForm<QDialog>
{
    public QVBoxLayout verticalLayout;
    public QToolBox toolBox;
    public QWidget page_2;
    public QGridLayout gridLayout_3;
    public QSpacerItem horizontalSpacer;
    public QWidget widget;
    public QGridLayout gridLayout;
    public QLabel label_3;
    public QLabel label_4;
    public QLabel label_5;
    public QLabel label_6;
    public QLabel label_2;
    public QLabel label_7;
    public QLabel label_8;
    public QLineEdit yaw_Kp;
    public QLineEdit yaw_Ki;
    public QLineEdit yaw_Kd;
    public QLineEdit yaw_scale;
    public QCheckBox yawEnabled;
    public QLabel yawActual;
    public QCheckBox pitchEnabled;
    public QLabel pitchActual;
    public QSpacerItem verticalSpacer_7;
    public QSpacerItem verticalSpacer_8;
    public QCheckBox depthEnabled;
    public QLabel depthActual;
    public QLineEdit pitch_Kp;
    public QLineEdit pitch_Ki;
    public QLineEdit pitch_Kd;
    public QLineEdit pitch_scale;
    public QLineEdit depth_Kp;
    public QLineEdit depth_Ki;
    public QLineEdit depth_Kd;
    public QLineEdit depth_scale;
    public QLineEdit yawTarget;
    public QLineEdit pitchTarget;
    public QLineEdit depthTarget;
    public QSpacerItem horizontalSpacer_2;
    public QSpacerItem verticalSpacer;
    public QSpacerItem verticalSpacer_2;
    public QWidget page_4;
    public QHBoxLayout horizontalLayout_2;
    public QSpacerItem horizontalSpacer_6;
    public QGridLayout gridLayout_2;
    public QDoubleSpinBox foreOffset;
    public QLabel label_11;
    public QLabel label_12;
    public QDoubleSpinBox foreScale;
    public QSpacerItem verticalSpacer_3;
    public QLabel label_13;
    public QLabel label_14;
    public QDoubleSpinBox aftScale;
    public QDoubleSpinBox aftOffset;
    public QSpacerItem horizontalSpacer_5;
    public QWidget page_3;
    public QHBoxLayout horizontalLayout_4;
    public QSpacerItem horizontalSpacer_8;
    public QGridLayout gridLayout_4;
    public QLabel label_9;
    public QSpinBox debugLevel;
    public QLabel label_10;
    public QSpacerItem verticalSpacer_4;
    public QCheckBox debugMessaging;
    public QSpacerItem verticalSpacer_5;
    public QPushButton resetMCB;
    public QSpacerItem horizontalSpacer_7;
    public QWidget page;
    public QHBoxLayout horizontalLayout;
    public QSpacerItem horizontalSpacer_3;
    public QWidget widget_2;
    public QLabel label;
    public QSpinBox gamepadID;
    public QSpacerItem horizontalSpacer_4;

    public Ui_Settings() { super(); }

    public void setupUi(QDialog Settings)
    {
        Settings.setObjectName("Settings");
        Settings.resize(new QSize(576, 407).expandedTo(Settings.minimumSizeHint()));
        Settings.setSizeGripEnabled(true);
        verticalLayout = new QVBoxLayout(Settings);
        verticalLayout.setObjectName("verticalLayout");
        toolBox = new QToolBox(Settings);
        toolBox.setObjectName("toolBox");
        page_2 = new QWidget();
        page_2.setObjectName("page_2");
        page_2.setGeometry(new QRect(0, 0, 558, 277));
        gridLayout_3 = new QGridLayout(page_2);
        gridLayout_3.setObjectName("gridLayout_3");
        horizontalSpacer = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout_3.addItem(horizontalSpacer, 1, 0, 1, 1);

        widget = new QWidget(page_2);
        widget.setObjectName("widget");
        gridLayout = new QGridLayout(widget);
        gridLayout.setObjectName("gridLayout");
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

        label_2 = new QLabel(widget);
        label_2.setObjectName("label_2");
        label_2.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_2, 1, 2, 1, 1);

        label_7 = new QLabel(widget);
        label_7.setObjectName("label_7");
        label_7.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_7, 7, 1, 1, 1);

        label_8 = new QLabel(widget);
        label_8.setObjectName("label_8");
        label_8.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label_8, 10, 1, 1, 1);

        yaw_Kp = new QLineEdit(widget);
        yaw_Kp.setObjectName("yaw_Kp");

        gridLayout.addWidget(yaw_Kp, 2, 2, 1, 1);

        yaw_Ki = new QLineEdit(widget);
        yaw_Ki.setObjectName("yaw_Ki");

        gridLayout.addWidget(yaw_Ki, 2, 3, 1, 1);

        yaw_Kd = new QLineEdit(widget);
        yaw_Kd.setObjectName("yaw_Kd");

        gridLayout.addWidget(yaw_Kd, 2, 4, 1, 1);

        yaw_scale = new QLineEdit(widget);
        yaw_scale.setObjectName("yaw_scale");

        gridLayout.addWidget(yaw_scale, 2, 5, 1, 1);

        yawEnabled = new QCheckBox(widget);
        yawEnabled.setObjectName("yawEnabled");

        gridLayout.addWidget(yawEnabled, 3, 2, 1, 1);

        yawActual = new QLabel(widget);
        yawActual.setObjectName("yawActual");

        gridLayout.addWidget(yawActual, 3, 5, 1, 1);

        pitchEnabled = new QCheckBox(widget);
        pitchEnabled.setObjectName("pitchEnabled");

        gridLayout.addWidget(pitchEnabled, 8, 2, 1, 1);

        pitchActual = new QLabel(widget);
        pitchActual.setObjectName("pitchActual");

        gridLayout.addWidget(pitchActual, 8, 5, 1, 1);

        verticalSpacer_7 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout.addItem(verticalSpacer_7, 9, 2, 1, 1);

        verticalSpacer_8 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout.addItem(verticalSpacer_8, 4, 2, 1, 1);

        depthEnabled = new QCheckBox(widget);
        depthEnabled.setObjectName("depthEnabled");

        gridLayout.addWidget(depthEnabled, 11, 2, 1, 1);

        depthActual = new QLabel(widget);
        depthActual.setObjectName("depthActual");
        depthActual.setMinimumSize(new QSize(150, 0));

        gridLayout.addWidget(depthActual, 11, 5, 1, 1);

        pitch_Kp = new QLineEdit(widget);
        pitch_Kp.setObjectName("pitch_Kp");

        gridLayout.addWidget(pitch_Kp, 7, 2, 1, 1);

        pitch_Ki = new QLineEdit(widget);
        pitch_Ki.setObjectName("pitch_Ki");

        gridLayout.addWidget(pitch_Ki, 7, 3, 1, 1);

        pitch_Kd = new QLineEdit(widget);
        pitch_Kd.setObjectName("pitch_Kd");

        gridLayout.addWidget(pitch_Kd, 7, 4, 1, 1);

        pitch_scale = new QLineEdit(widget);
        pitch_scale.setObjectName("pitch_scale");

        gridLayout.addWidget(pitch_scale, 7, 5, 1, 1);

        depth_Kp = new QLineEdit(widget);
        depth_Kp.setObjectName("depth_Kp");

        gridLayout.addWidget(depth_Kp, 10, 2, 1, 1);

        depth_Ki = new QLineEdit(widget);
        depth_Ki.setObjectName("depth_Ki");

        gridLayout.addWidget(depth_Ki, 10, 3, 1, 1);

        depth_Kd = new QLineEdit(widget);
        depth_Kd.setObjectName("depth_Kd");

        gridLayout.addWidget(depth_Kd, 10, 4, 1, 1);

        depth_scale = new QLineEdit(widget);
        depth_scale.setObjectName("depth_scale");

        gridLayout.addWidget(depth_scale, 10, 5, 1, 1);

        yawTarget = new QLineEdit(widget);
        yawTarget.setObjectName("yawTarget");

        gridLayout.addWidget(yawTarget, 3, 3, 1, 1);

        pitchTarget = new QLineEdit(widget);
        pitchTarget.setObjectName("pitchTarget");

        gridLayout.addWidget(pitchTarget, 8, 3, 1, 1);

        depthTarget = new QLineEdit(widget);
        depthTarget.setObjectName("depthTarget");

        gridLayout.addWidget(depthTarget, 11, 3, 1, 1);


        gridLayout_3.addWidget(widget, 1, 1, 1, 1);

        horizontalSpacer_2 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        gridLayout_3.addItem(horizontalSpacer_2, 1, 2, 1, 1);

        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout_3.addItem(verticalSpacer, 0, 1, 1, 1);

        verticalSpacer_2 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout_3.addItem(verticalSpacer_2, 2, 1, 1, 1);

        toolBox.addItem(page_2, com.trolltech.qt.core.QCoreApplication.translate("Settings", "Autopilot Settings", null));
        page_4 = new QWidget();
        page_4.setObjectName("page_4");
        page_4.setGeometry(new QRect(0, 0, 297, 72));
        horizontalLayout_2 = new QHBoxLayout(page_4);
        horizontalLayout_2.setObjectName("horizontalLayout_2");
        horizontalSpacer_6 = new QSpacerItem(143, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_2.addItem(horizontalSpacer_6);

        gridLayout_2 = new QGridLayout();
        gridLayout_2.setObjectName("gridLayout_2");
        foreOffset = new QDoubleSpinBox(page_4);
        foreOffset.setObjectName("foreOffset");
        foreOffset.setMinimum(-10000);
        foreOffset.setMaximum(1000);
        foreOffset.setSingleStep(0.01);

        gridLayout_2.addWidget(foreOffset, 0, 1, 1, 1);

        label_11 = new QLabel(page_4);
        label_11.setObjectName("label_11");

        gridLayout_2.addWidget(label_11, 0, 0, 1, 1);

        label_12 = new QLabel(page_4);
        label_12.setObjectName("label_12");

        gridLayout_2.addWidget(label_12, 1, 0, 1, 1);

        foreScale = new QDoubleSpinBox(page_4);
        foreScale.setObjectName("foreScale");
        foreScale.setMinimum(-10000);
        foreScale.setMaximum(1000);
        foreScale.setSingleStep(0.01);

        gridLayout_2.addWidget(foreScale, 1, 1, 1, 1);

        verticalSpacer_3 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout_2.addItem(verticalSpacer_3, 4, 1, 1, 1);

        label_13 = new QLabel(page_4);
        label_13.setObjectName("label_13");

        gridLayout_2.addWidget(label_13, 0, 2, 1, 1);

        label_14 = new QLabel(page_4);
        label_14.setObjectName("label_14");

        gridLayout_2.addWidget(label_14, 1, 2, 1, 1);

        aftScale = new QDoubleSpinBox(page_4);
        aftScale.setObjectName("aftScale");
        aftScale.setMinimum(-10000);
        aftScale.setMaximum(1000);
        aftScale.setSingleStep(0.01);

        gridLayout_2.addWidget(aftScale, 1, 3, 1, 1);

        aftOffset = new QDoubleSpinBox(page_4);
        aftOffset.setObjectName("aftOffset");
        aftOffset.setMinimum(-10000);
        aftOffset.setMaximum(1000);
        aftOffset.setSingleStep(0.01);

        gridLayout_2.addWidget(aftOffset, 0, 3, 1, 1);


        horizontalLayout_2.addLayout(gridLayout_2);

        horizontalSpacer_5 = new QSpacerItem(142, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_2.addItem(horizontalSpacer_5);

        toolBox.addItem(page_4, com.trolltech.qt.core.QCoreApplication.translate("Settings", "Depth Calibration", null));
        page_3 = new QWidget();
        page_3.setObjectName("page_3");
        page_3.setGeometry(new QRect(0, 0, 558, 277));
        horizontalLayout_4 = new QHBoxLayout(page_3);
        horizontalLayout_4.setObjectName("horizontalLayout_4");
        horizontalSpacer_8 = new QSpacerItem(184, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_4.addItem(horizontalSpacer_8);

        gridLayout_4 = new QGridLayout();
        gridLayout_4.setObjectName("gridLayout_4");
        label_9 = new QLabel(page_3);
        label_9.setObjectName("label_9");

        gridLayout_4.addWidget(label_9, 1, 0, 1, 1);

        debugLevel = new QSpinBox(page_3);
        debugLevel.setObjectName("debugLevel");
        QSizePolicy sizePolicy = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Fixed, com.trolltech.qt.gui.QSizePolicy.Policy.Fixed);
        sizePolicy.setHorizontalStretch((byte)0);
        sizePolicy.setVerticalStretch((byte)0);
        sizePolicy.setHeightForWidth(debugLevel.sizePolicy().hasHeightForWidth());
        debugLevel.setSizePolicy(sizePolicy);
        debugLevel.setMinimum(-10000);
        debugLevel.setMaximum(10000);
        debugLevel.setValue(1);

        gridLayout_4.addWidget(debugLevel, 1, 1, 1, 1);

        label_10 = new QLabel(page_3);
        label_10.setObjectName("label_10");

        gridLayout_4.addWidget(label_10, 2, 0, 1, 1);

        verticalSpacer_4 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout_4.addItem(verticalSpacer_4, 5, 1, 1, 1);

        debugMessaging = new QCheckBox(page_3);
        debugMessaging.setObjectName("debugMessaging");

        gridLayout_4.addWidget(debugMessaging, 2, 1, 1, 1);

        verticalSpacer_5 = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        gridLayout_4.addItem(verticalSpacer_5, 0, 1, 1, 1);

        resetMCB = new QPushButton(page_3);
        resetMCB.setObjectName("resetMCB");

        gridLayout_4.addWidget(resetMCB, 3, 1, 1, 1);


        horizontalLayout_4.addLayout(gridLayout_4);

        horizontalSpacer_7 = new QSpacerItem(183, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout_4.addItem(horizontalSpacer_7);

        toolBox.addItem(page_3, com.trolltech.qt.core.QCoreApplication.translate("Settings", "Debug Settings", null));
        page = new QWidget();
        page.setObjectName("page");
        page.setGeometry(new QRect(0, 0, 230, 38));
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

        horizontalLayout.addWidget(widget_2);

        horizontalSpacer_4 = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout.addItem(horizontalSpacer_4);

        toolBox.addItem(page, com.trolltech.qt.core.QCoreApplication.translate("Settings", "Gamepad Settings", null));

        verticalLayout.addWidget(toolBox);

        retranslateUi(Settings);

        toolBox.setCurrentIndex(2);


        Settings.connectSlotsByName();
    } // setupUi

    void retranslateUi(QDialog Settings)
    {
        Settings.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Dialog", null));
        label_3.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Ki", null));
        label_4.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Kd", null));
        label_5.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "scale", null));
        label_6.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Yaw", null));
        label_2.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Kp", null));
        label_7.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Pitch", null));
        label_8.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Depth", null));
        yaw_Kp.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "1", null));
        yaw_Ki.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        yaw_Kd.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "-40", null));
        yaw_scale.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "1", null));
        yawEnabled.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Enabled", null));
        yawActual.setText("");
        pitchEnabled.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Enabled", null));
        pitchActual.setText("");
        depthEnabled.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Enabled", null));
        depthActual.setText("");
        pitch_Kp.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        pitch_Ki.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        pitch_Kd.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        pitch_scale.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        depth_Kp.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "40", null));
        depth_Ki.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        depth_Kd.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        depth_scale.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "1", null));
        yawTarget.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        pitchTarget.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        depthTarget.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "0", null));
        toolBox.setItemText(toolBox.indexOf(page_2), com.trolltech.qt.core.QCoreApplication.translate("Settings", "Autopilot Settings", null));
        label_11.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Fore Offset", null));
        label_12.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Fore Scale", null));
        label_13.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Aft Offset", null));
        label_14.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Aft Scale", null));
        toolBox.setItemText(toolBox.indexOf(page_4), com.trolltech.qt.core.QCoreApplication.translate("Settings", "Depth Calibration", null));
        label_9.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Debug Level ", null));
        label_10.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Debug Messaging", null));
        debugMessaging.setText("");
        resetMCB.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Reset MCB", null));
        toolBox.setItemText(toolBox.indexOf(page_3), com.trolltech.qt.core.QCoreApplication.translate("Settings", "Debug Settings", null));
        label.setText(com.trolltech.qt.core.QCoreApplication.translate("Settings", "Gamepad ID", null));
        toolBox.setItemText(toolBox.indexOf(page), com.trolltech.qt.core.QCoreApplication.translate("Settings", "Gamepad Settings", null));
    } // retranslateUi

}

