/********************************************************************************
** Form generated from reading ui file 'CameraFeeds.jui'
**
** Created: Tue 22. Jun 17:01:06 2010
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
    public VideoScreen forwardCamIcon;
    public VideoScreen downwardCamIcon;
    public VideoScreen sonarIcon;
    public QSpacerItem verticalSpacer;
    public QStackedWidget feeds;
    public QWidget forwardCamPage;
    public QHBoxLayout horizontalLayout_2;
    public ControlableVideoScreen forwardCam;
    public QWidget downwardCamPage;
    public QHBoxLayout horizontalLayout_3;
    public ControlableVideoScreen downwardCam;
    public QWidget sonarCamPage;
    public QHBoxLayout horizontalLayout_4;
    public ControlableVideoScreen sonarCam;

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
        forwardCamIcon = new VideoScreen(scrollAreaWidgetContents);
        forwardCamIcon.setObjectName("forwardCamIcon");
        forwardCamIcon.setMinimumSize(new QSize(0, 90));
        forwardCamIcon.setMaximumSize(new QSize(16777215, 90));
        forwardCamIcon.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));

        verticalLayout.addWidget(forwardCamIcon);

        downwardCamIcon = new VideoScreen(scrollAreaWidgetContents);
        downwardCamIcon.setObjectName("downwardCamIcon");
        downwardCamIcon.setMinimumSize(new QSize(0, 90));
        downwardCamIcon.setMaximumSize(new QSize(16777215, 90));
        downwardCamIcon.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));

        verticalLayout.addWidget(downwardCamIcon);

        sonarIcon = new VideoScreen(scrollAreaWidgetContents);
        sonarIcon.setObjectName("sonarIcon");
        sonarIcon.setMinimumSize(new QSize(0, 90));
        sonarIcon.setMaximumSize(new QSize(16777215, 90));
        sonarIcon.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));

        verticalLayout.addWidget(sonarIcon);

        verticalSpacer = new QSpacerItem(20, 40, com.trolltech.qt.gui.QSizePolicy.Policy.Minimum, com.trolltech.qt.gui.QSizePolicy.Policy.Expanding);

        verticalLayout.addItem(verticalSpacer);

        scrollArea.setWidget(scrollAreaWidgetContents);

        horizontalLayout.addWidget(scrollArea);

        feeds = new QStackedWidget(CameraFeeds);
        feeds.setObjectName("feeds");
        forwardCamPage = new QWidget();
        forwardCamPage.setObjectName("forwardCamPage");
        horizontalLayout_2 = new QHBoxLayout(forwardCamPage);
        horizontalLayout_2.setMargin(0);
        horizontalLayout_2.setObjectName("horizontalLayout_2");
        forwardCam = new ControlableVideoScreen(forwardCamPage);
        forwardCam.setObjectName("forwardCam");

        horizontalLayout_2.addWidget(forwardCam);

        feeds.addWidget(forwardCamPage);
        downwardCamPage = new QWidget();
        downwardCamPage.setObjectName("downwardCamPage");
        horizontalLayout_3 = new QHBoxLayout(downwardCamPage);
        horizontalLayout_3.setSpacing(0);
        horizontalLayout_3.setMargin(0);
        horizontalLayout_3.setObjectName("horizontalLayout_3");
        downwardCam = new ControlableVideoScreen(downwardCamPage);
        downwardCam.setObjectName("downwardCam");

        horizontalLayout_3.addWidget(downwardCam);

        feeds.addWidget(downwardCamPage);
        sonarCamPage = new QWidget();
        sonarCamPage.setObjectName("sonarCamPage");
        horizontalLayout_4 = new QHBoxLayout(sonarCamPage);
        horizontalLayout_4.setSpacing(0);
        horizontalLayout_4.setMargin(0);
        horizontalLayout_4.setObjectName("horizontalLayout_4");
        sonarCam = new ControlableVideoScreen(sonarCamPage);
        sonarCam.setObjectName("sonarCam");

        horizontalLayout_4.addWidget(sonarCam);

        feeds.addWidget(sonarCamPage);

        horizontalLayout.addWidget(feeds);

        retranslateUi(CameraFeeds);

        feeds.setCurrentIndex(2);


        CameraFeeds.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget CameraFeeds)
    {
        CameraFeeds.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "Form", null));
    } // retranslateUi

}

