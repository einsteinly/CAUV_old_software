/********************************************************************************
** Form generated from reading ui file 'MissionControlView.jui'
**
** Created: Wed 5. May 15:47:53 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui.views;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_MissionControlView implements com.trolltech.qt.QUiForm<QWidget>
{
    public QVBoxLayout verticalLayout;
    public QWidget widget;
    public QHBoxLayout horizontalLayout;
    public QPushButton newFile;
    public QPushButton openFile;
    public QPushButton saveButton;
    public QPushButton saveAsButton;
    public QSpacerItem horizontalSpacer;
    public QTabWidget openFiles;
    public QWidget tab;
    public QVBoxLayout verticalLayout_2;
    public QLabel label;

    public Ui_MissionControlView() { super(); }

    public void setupUi(QWidget MissionControlView)
    {
        MissionControlView.setObjectName("MissionControlView");
        MissionControlView.resize(new QSize(496, 321).expandedTo(MissionControlView.minimumSizeHint()));
        verticalLayout = new QVBoxLayout(MissionControlView);
        verticalLayout.setMargin(0);
        verticalLayout.setObjectName("verticalLayout");
        widget = new QWidget(MissionControlView);
        widget.setObjectName("widget");
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout.setSpacing(0);
        horizontalLayout.setMargin(0);
        horizontalLayout.setObjectName("horizontalLayout");
        newFile = new QPushButton(widget);
        newFile.setObjectName("newFile");
        newFile.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-new.png")));
        newFile.setIconSize(new QSize(22, 22));
        newFile.setFlat(true);

        horizontalLayout.addWidget(newFile);

        openFile = new QPushButton(widget);
        openFile.setObjectName("openFile");
        openFile.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-open.png")));
        openFile.setIconSize(new QSize(22, 22));
        openFile.setFlat(true);

        horizontalLayout.addWidget(openFile);

        saveButton = new QPushButton(widget);
        saveButton.setObjectName("saveButton");
        saveButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-save.png")));
        saveButton.setIconSize(new QSize(22, 22));
        saveButton.setFlat(true);

        horizontalLayout.addWidget(saveButton);

        saveAsButton = new QPushButton(widget);
        saveAsButton.setObjectName("saveAsButton");
        saveAsButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-save-as.png")));
        saveAsButton.setIconSize(new QSize(22, 22));
        saveAsButton.setFlat(true);

        horizontalLayout.addWidget(saveAsButton);

        horizontalSpacer = new QSpacerItem(40, 20, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum);

        horizontalLayout.addItem(horizontalSpacer);


        verticalLayout.addWidget(widget);

        openFiles = new QTabWidget(MissionControlView);
        openFiles.setObjectName("openFiles");
        openFiles.setTabPosition(com.trolltech.qt.gui.QTabWidget.TabPosition.North);
        openFiles.setTabShape(com.trolltech.qt.gui.QTabWidget.TabShape.Rounded);
        openFiles.setElideMode(com.trolltech.qt.core.Qt.TextElideMode.ElideRight);
        openFiles.setDocumentMode(true);
        openFiles.setTabsClosable(true);
        openFiles.setMovable(true);
        tab = new QWidget();
        tab.setObjectName("tab");
        verticalLayout_2 = new QVBoxLayout(tab);
        verticalLayout_2.setSpacing(0);
        verticalLayout_2.setMargin(0);
        verticalLayout_2.setObjectName("verticalLayout_2");
        label = new QLabel(tab);
        label.setObjectName("label");
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
        label.setPalette(palette);
        label.setAutoFillBackground(true);
        label.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        verticalLayout_2.addWidget(label);

        openFiles.addTab(tab, "");

        verticalLayout.addWidget(openFiles);

        retranslateUi(MissionControlView);

        openFiles.setCurrentIndex(0);


        MissionControlView.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget MissionControlView)
    {
        newFile.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "New", null));
        newFile.setText("");
        openFile.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Open", null));
        openFile.setText("");
        saveButton.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Save", null));
        saveButton.setText("");
        saveAsButton.setToolTip(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Save As", null));
        saveAsButton.setText("");
        label.setText(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Open or create a file to use the mission editor.", null));
        openFiles.setTabText(openFiles.indexOf(tab), "");
    } // retranslateUi

}

