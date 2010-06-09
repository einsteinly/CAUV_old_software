/********************************************************************************
** Form generated from reading ui file 'MissionControlView.jui'
**
** Created: Wed 17. Mar 17:17:37 2010
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
    public QPushButton runButton;
    public QSpacerItem horizontalSpacer;
    public QTabWidget openFiles;
    public QWidget tab;
    public QGridLayout gridLayout;
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
        QPalette palette= new QPalette();
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.AlternateBase, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Active, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.WindowText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Text, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ButtonText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Base, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.AlternateBase, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Inactive, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Button, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Light, new QColor(49, 49, 49));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Midlight, new QColor(41, 41, 41));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Dark, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Mid, new QColor(22, 22, 22));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.BrightText, new QColor(255, 255, 255));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, new QColor(16, 16, 16));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Base, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Window, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Shadow, new QColor(0, 0, 0));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.AlternateBase, new QColor(33, 33, 33));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipBase, new QColor(255, 255, 220));
        palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ToolTipText, new QColor(0, 0, 0));
        widget.setPalette(palette);
        widget.setAutoFillBackground(true);
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout.setSpacing(0);
        horizontalLayout.setMargin(0);
        horizontalLayout.setObjectName("horizontalLayout");
        newFile = new QPushButton(widget);
        newFile.setObjectName("newFile");
        newFile.setAutoFillBackground(false);
        newFile.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-new.png")));
        newFile.setIconSize(new QSize(20, 20));
        newFile.setFlat(true);

        horizontalLayout.addWidget(newFile);

        openFile = new QPushButton(widget);
        openFile.setObjectName("openFile");
        openFile.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-open.png")));
        openFile.setIconSize(new QSize(20, 20));
        openFile.setFlat(true);

        horizontalLayout.addWidget(openFile);

        saveButton = new QPushButton(widget);
        saveButton.setObjectName("saveButton");
        saveButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-save.png")));
        saveButton.setIconSize(new QSize(20, 20));
        saveButton.setFlat(true);

        horizontalLayout.addWidget(saveButton);

        saveAsButton = new QPushButton(widget);
        saveAsButton.setObjectName("saveAsButton");
        saveAsButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/document-save-as.png")));
        saveAsButton.setIconSize(new QSize(20, 20));
        saveAsButton.setFlat(true);

        horizontalLayout.addWidget(saveAsButton);

        runButton = new QPushButton(widget);
        runButton.setObjectName("runButton");
        runButton.setIcon(new QIcon(new QPixmap("classpath:cauv/gui/resources/icon.png")));
        runButton.setIconSize(new QSize(20, 20));
        runButton.setFlat(true);

        horizontalLayout.addWidget(runButton);

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
        tab.setAutoFillBackground(true);
        gridLayout = new QGridLayout(tab);
        gridLayout.setObjectName("gridLayout");
        label = new QLabel(tab);
        label.setObjectName("label");
        label.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignCenter));

        gridLayout.addWidget(label, 0, 0, 1, 1);

        openFiles.addTab(tab, "");

        verticalLayout.addWidget(openFiles);

        retranslateUi(MissionControlView);

        openFiles.setCurrentIndex(0);


        MissionControlView.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget MissionControlView)
    {
        MissionControlView.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Form", null));
        newFile.setText(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "New", null));
        openFile.setText(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Open", null));
        saveButton.setText(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Save", null));
        saveAsButton.setText(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Save As", null));
        runButton.setText(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Run Mission", null));
        label.setText(com.trolltech.qt.core.QCoreApplication.translate("MissionControlView", "Open or create a file to use the mission editor.", null));
        openFiles.setTabText(openFiles.indexOf(tab), "");
    } // retranslateUi

}

