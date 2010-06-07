/********************************************************************************
** Form generated from reading ui file 'CameraFeeds.jui'
**
** Created: Wed 5. May 13:23:01 2010
**      by: Qt User Interface Compiler version 4.5.2
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

package cauv.gui.views;

import com.trolltech.qt.core.*;
import com.trolltech.qt.gui.*;

import cauv.gui.components.*;

public class Ui_CameraFeeds implements com.trolltech.qt.QUiForm<QWidget>
{
    public QHBoxLayout horizontalLayout;
    public QScrollArea scrollArea;
    public QWidget scrollAreaWidgetContents;
    public QVBoxLayout verticalLayout;
    public VideoScreen videoScreen_2;
    public QSpacerItem verticalSpacer;
    public QStackedWidget stackedWidget;
    public QWidget page;
    public QHBoxLayout horizontalLayout_2;
    public ControlableVideoScreen controlableVideoScreen;
    public QWidget page_2;

    public Ui_CameraFeeds() { super(); }

    public void setupUi(QWidget CameraFeeds)
    {
        CameraFeeds.setObjectName("CameraFeeds");
        CameraFeeds.resize(new QSize(614, 377).expandedTo(CameraFeeds.minimumSizeHint()));
        horizontalLayout = new QHBoxLayout(CameraFeeds);
        horizontalLayout.setObjectName("horizontalLayout");
        scrollArea = new QScrollArea(CameraFeeds);
        scrollArea.setObjectName("scrollArea");
        QSizePolicy sizePolicy = new QSizePolicy(com.trolltech.qt.gui.QSizePolicy.Policy.Fixed, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);
        sizePolicy.setHorizontalStretch((byte)0);
        sizePolicy.setVerticalStretch((byte)0);
        sizePolicy.setHeightForWidth(scrollArea.sizePolicy().hasHeightForWidth());
        scrollArea.setSizePolicy(sizePolicy);
        scrollArea.setMinimumSize(new QSize(130, 0));
        scrollArea.setFrameShape(com.trolltech.qt.gui.QFrame.Shape.Box);
        scrollArea.setLineWidth(0);
        scrollArea.setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents");
        scrollAreaWidgetContents.setGeometry(new QRect(0, 0, 130, 359));
        verticalLayout = new QVBoxLayout(scrollAreaWidgetContents);
        verticalLayout.setObjectName("verticalLayout");
        videoScreen_2 = new VideoScreen(scrollAreaWidgetContents);
        videoScreen_2.setObjectName("videoScreen_2");
        videoScreen_2.setMinimumSize(new QSize(0, 90));
        videoScreen_2.setMaximumSize(new QSize(16777215, 90));
        videoScreen_2.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));

        verticalLayout.addWidget(videoScreen_2);

        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        verticalLayout.addItem(verticalSpacer);

        scrollArea.setWidget(scrollAreaWidgetContents);

        horizontalLayout.addWidget(scrollArea);

        stackedWidget = new QStackedWidget(CameraFeeds);
        stackedWidget.setObjectName("stackedWidget");
        page = new QWidget();
        page.setObjectName("page");
        horizontalLayout_2 = new QHBoxLayout(page);
        horizontalLayout_2.setMargin(0);
        horizontalLayout_2.setObjectName("horizontalLayout_2");
        controlableVideoScreen = new ControlableVideoScreen(page);
        controlableVideoScreen.setObjectName("controlableVideoScreen");

        horizontalLayout_2.addWidget(controlableVideoScreen);

        stackedWidget.addWidget(page);
        page_2 = new QWidget();
        page_2.setObjectName("page_2");
        stackedWidget.addWidget(page_2);

        horizontalLayout.addWidget(stackedWidget);

        retranslateUi(CameraFeeds);

        CameraFeeds.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget CameraFeeds)
    {
        CameraFeeds.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "Form", null));
    } // retranslateUi

}

