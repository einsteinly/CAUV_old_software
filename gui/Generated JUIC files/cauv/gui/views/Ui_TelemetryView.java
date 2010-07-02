/********************************************************************************
** Form generated from reading ui file 'TelemetryView.jui'
**
** Created: Sat 3. Jul 00:01:54 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui.views;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

public class Ui_TelemetryView implements com.trolltech.qt.QUiForm<QWidget>
{
    public QHBoxLayout horizontalLayout;
    public QMdiArea graphs;
    public QListWidget listWidget;

    public Ui_TelemetryView() { super(); }

    public void setupUi(QWidget TelemetryView)
    {
        TelemetryView.setObjectName("TelemetryView");
        TelemetryView.resize(new QSize(570, 337).expandedTo(TelemetryView.minimumSizeHint()));
        horizontalLayout = new QHBoxLayout(TelemetryView);
        horizontalLayout.setObjectName("horizontalLayout");
        graphs = new QMdiArea(TelemetryView);
        graphs.setObjectName("graphs");
        graphs.setVerticalScrollBarPolicy(com.trolltech.qt.core.Qt.ScrollBarPolicy.ScrollBarAsNeeded);
        graphs.setHorizontalScrollBarPolicy(com.trolltech.qt.core.Qt.ScrollBarPolicy.ScrollBarAsNeeded);

        horizontalLayout.addWidget(graphs);

        listWidget = new QListWidget(TelemetryView);
        listWidget.setObjectName("listWidget");
        listWidget.setMinimumSize(new QSize(130, 0));
        listWidget.setDragDropMode(com.trolltech.qt.gui.QAbstractItemView.DragDropMode.NoDragDrop);
        listWidget.setAlternatingRowColors(true);
        listWidget.setSelectionMode(com.trolltech.qt.gui.QAbstractItemView.SelectionMode.MultiSelection);
        listWidget.setSortingEnabled(true);

        horizontalLayout.addWidget(listWidget);

        retranslateUi(TelemetryView);

        TelemetryView.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget TelemetryView)
    {
        TelemetryView.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("TelemetryView", "Form", null));
    } // retranslateUi

}

