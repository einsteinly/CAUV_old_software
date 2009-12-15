/********************************************************************************
** Form generated from reading ui file 'ScreenIcon.jui'
**
** Created: Sat 12. Dec 20:47:12 2009
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_ScreenIcon implements com.trolltech.qt.QUiForm<QWidget>
{
    public QVBoxLayout verticalLayout;
    public QWidget widget;
    public QHBoxLayout horizontalLayout;
    public QGridLayout iconLayout;
    public QLabel iconLabel;

    public Ui_ScreenIcon() { super(); }

    public void setupUi(QWidget ScreenIcon)
    {
        ScreenIcon.setObjectName("ScreenIcon");
        ScreenIcon.resize(new QSize(92, 92).expandedTo(ScreenIcon.minimumSizeHint()));
        ScreenIcon.setMinimumSize(new QSize(0, 0));
        ScreenIcon.setMaximumSize(new QSize(102, 102));
        ScreenIcon.setAutoFillBackground(true);
        verticalLayout = new QVBoxLayout(ScreenIcon);
        verticalLayout.setMargin(0);
        verticalLayout.setObjectName("verticalLayout");
        widget = new QWidget(ScreenIcon);
        widget.setObjectName("widget");
        widget.setMinimumSize(new QSize(92, 72));
        widget.setMaximumSize(new QSize(92, 72));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout.setSpacing(0);
        horizontalLayout.setMargin(0);
        horizontalLayout.setObjectName("horizontalLayout");
        iconLayout = new QGridLayout();
        iconLayout.setSpacing(0);
        iconLayout.setObjectName("iconLayout");

        horizontalLayout.addLayout(iconLayout);


        verticalLayout.addWidget(widget);

        iconLabel = new QLabel(ScreenIcon);
        iconLabel.setObjectName("iconLabel");
        iconLabel.setMaximumSize(new QSize(92, 14));
        iconLabel.setStyleSheet("#iconLabel{\n"+
"	font-size: 11px;\n"+
"	font-weight: bold;\n"+
"	color: white;\n"+
"}");
        iconLabel.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        verticalLayout.addWidget(iconLabel);

        retranslateUi(ScreenIcon);

        ScreenIcon.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget ScreenIcon)
    {
        ScreenIcon.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("ScreenIcon", "Form", null));
        iconLabel.setText(com.trolltech.qt.core.QCoreApplication.translate("ScreenIcon", "TextLabel", null));
    } // retranslateUi

}

