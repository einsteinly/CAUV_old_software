/********************************************************************************
** Form generated from reading ui file 'ControlableVideoScreen.jui'
**
** Created: Thu 6. May 00:20:22 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui.components;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_ControlableVideoScreen implements com.trolltech.qt.QUiForm<QWidget>
{
    public QGridLayout gridLayout;
    public QPushButton up;
    public QPushButton left;
    public QWidget widget;
    public QVBoxLayout verticalLayout;
    public QLabel label;
    public QSpacerItem verticalSpacer;
    public QPushButton right;
    public QPushButton down;

    public Ui_ControlableVideoScreen() { super(); }

    public void setupUi(QWidget ControlableVideoScreen)
    {
        ControlableVideoScreen.setObjectName("ControlableVideoScreen");
        ControlableVideoScreen.resize(new QSize(400, 300).expandedTo(ControlableVideoScreen.minimumSizeHint()));
        gridLayout = new QGridLayout(ControlableVideoScreen);
        gridLayout.setObjectName("gridLayout");
        up = new QPushButton(ControlableVideoScreen);
        up.setObjectName("up");
        QSizePolicy sizePolicy = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Fixed);
        sizePolicy.setHorizontalStretch((byte)0);
        sizePolicy.setVerticalStretch((byte)0);
        sizePolicy.setHeightForWidth(up.sizePolicy().hasHeightForWidth());
        up.setSizePolicy(sizePolicy);
        up.setMinimumSize(new QSize(0, 25));
        up.setMaximumSize(new QSize(16777215, 25));
        up.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));
        up.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/arrow-up.png")));

        gridLayout.addWidget(up, 0, 1, 1, 1);

        left = new QPushButton(ControlableVideoScreen);
        left.setObjectName("left");
        QSizePolicy sizePolicy1 = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Fixed, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);
        sizePolicy1.setHorizontalStretch((byte)0);
        sizePolicy1.setVerticalStretch((byte)0);
        sizePolicy1.setHeightForWidth(left.sizePolicy().hasHeightForWidth());
        left.setSizePolicy(sizePolicy1);
        left.setMinimumSize(new QSize(25, 0));
        left.setMaximumSize(new QSize(25, 16777215));
        left.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));
        left.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/arrow-left.png")));

        gridLayout.addWidget(left, 1, 0, 1, 1);

        widget = new QWidget(ControlableVideoScreen);
        widget.setObjectName("widget");
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout.setMargin(0);
        verticalLayout.setObjectName("verticalLayout");
        label = new QLabel(widget);
        label.setObjectName("label");
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
        label.setPalette(palette);
        QFont font = new QFont();
        font.setFamily("Consolas");
        font.setPointSize(8);
        font.setBold(true);
        font.setWeight(75);
        label.setFont(font);
        label.setTextFormat(com.trolltech.qt.core.Qt.TextFormat.AutoText);

        verticalLayout.addWidget(label);

        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        verticalLayout.addItem(verticalSpacer);


        gridLayout.addWidget(widget, 1, 1, 1, 1);

        right = new QPushButton(ControlableVideoScreen);
        right.setObjectName("right");
        QSizePolicy sizePolicy2 = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Fixed, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);
        sizePolicy2.setHorizontalStretch((byte)0);
        sizePolicy2.setVerticalStretch((byte)0);
        sizePolicy2.setHeightForWidth(right.sizePolicy().hasHeightForWidth());
        right.setSizePolicy(sizePolicy2);
        right.setMinimumSize(new QSize(25, 0));
        right.setMaximumSize(new QSize(25, 16777215));
        right.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));
        right.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/arrow-right.png")));
        right.setFlat(false);

        gridLayout.addWidget(right, 1, 2, 1, 1);

        down = new QPushButton(ControlableVideoScreen);
        down.setObjectName("down");
        QSizePolicy sizePolicy3 = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Fixed);
        sizePolicy3.setHorizontalStretch((byte)0);
        sizePolicy3.setVerticalStretch((byte)0);
        sizePolicy3.setHeightForWidth(down.sizePolicy().hasHeightForWidth());
        down.setSizePolicy(sizePolicy3);
        down.setMinimumSize(new QSize(0, 25));
        down.setMaximumSize(new QSize(16777215, 25));
        down.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));
        down.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icons/arrow-down.png")));

        gridLayout.addWidget(down, 2, 1, 1, 1);

        retranslateUi(ControlableVideoScreen);

        ControlableVideoScreen.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget ControlableVideoScreen)
    {
        ControlableVideoScreen.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("ControlableVideoScreen", "Form", null));
        ControlableVideoScreen.setStyleSheet(com.trolltech.qt.core.QCoreApplication.translate("ControlableVideoScreen", "QPushButton {\n"+
"background-color: rgba(226, 231, 231, 140);\n"+
"border-style: none;\n"+
"padding: 4px;\n"+
"}\n"+
"\n"+
"QPushButton:pressed {\n"+
"background-color: rgba(226, 231, 231, 200);\n"+
"border-style: none;\n"+
"padding: 4px;\n"+
"}", null));
        up.setText("");
        left.setText("");
        label.setText("");
        right.setText("");
        down.setText("");
    } // retranslateUi

}

