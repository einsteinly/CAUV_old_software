/********************************************************************************
** Form generated from reading ui file 'CameraFeeds.jui'
**
** Created: Sun 27. Jun 22:47:57 2010
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
    public QWidget widget;
    public QVBoxLayout verticalLayout_2;
    public ControlableVideoScreen sonarCam;
    public QWidget widget_2;
    public QHBoxLayout horizontalLayout_5;
    public QLabel label;
    public QSpinBox direction;
    public QLabel label_2;
    public QSpinBox width;
    public QLabel label_3;
    public QSpinBox gain;
    public QLabel label_4;
    public QSpinBox range;
    public QLabel label_5;
    public QSpinBox radialRes;
    public QLabel label_6;
    public QSpinBox angularRes;

    public Ui_CameraFeeds() { super(); }

    public void setupUi(QWidget CameraFeeds)
    {
        CameraFeeds.setObjectName("CameraFeeds");
        CameraFeeds.resize(new QSize(714, 377).expandedTo(CameraFeeds.minimumSizeHint()));
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
        widget = new QWidget(sonarCamPage);
        widget.setObjectName("widget");
        verticalLayout_2 = new QVBoxLayout(widget);
        verticalLayout_2.setSpacing(0);
        verticalLayout_2.setMargin(0);
        verticalLayout_2.setObjectName("verticalLayout_2");
        sonarCam = new ControlableVideoScreen(widget);
        sonarCam.setObjectName("sonarCam");

        verticalLayout_2.addWidget(sonarCam);

        widget_2 = new QWidget(widget);
        widget_2.setObjectName("widget_2");
        horizontalLayout_5 = new QHBoxLayout(widget_2);
        horizontalLayout_5.setObjectName("horizontalLayout_5");
        horizontalLayout_5.setContentsMargins(-1, 9, -1, -1);
        label = new QLabel(widget_2);
        label.setObjectName("label");
        label.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        horizontalLayout_5.addWidget(label);

        direction = new QSpinBox(widget_2);
        direction.setObjectName("direction");

        horizontalLayout_5.addWidget(direction);

        label_2 = new QLabel(widget_2);
        label_2.setObjectName("label_2");
        label_2.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        horizontalLayout_5.addWidget(label_2);

        width = new QSpinBox(widget_2);
        width.setObjectName("width");

        horizontalLayout_5.addWidget(width);

        label_3 = new QLabel(widget_2);
        label_3.setObjectName("label_3");
        label_3.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        horizontalLayout_5.addWidget(label_3);

        gain = new QSpinBox(widget_2);
        gain.setObjectName("gain");

        horizontalLayout_5.addWidget(gain);

        label_4 = new QLabel(widget_2);
        label_4.setObjectName("label_4");
        label_4.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        horizontalLayout_5.addWidget(label_4);

        range = new QSpinBox(widget_2);
        range.setObjectName("range");

        horizontalLayout_5.addWidget(range);

        label_5 = new QLabel(widget_2);
        label_5.setObjectName("label_5");
        label_5.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        horizontalLayout_5.addWidget(label_5);

        radialRes = new QSpinBox(widget_2);
        radialRes.setObjectName("radialRes");

        horizontalLayout_5.addWidget(radialRes);

        label_6 = new QLabel(widget_2);
        label_6.setObjectName("label_6");
        label_6.setAlignment(com.trolltech.qt.core.Qt.AlignmentFlag.createQFlags(com.trolltech.qt.core.Qt.AlignmentFlag.AlignRight,com.trolltech.qt.core.Qt.AlignmentFlag.AlignVCenter));

        horizontalLayout_5.addWidget(label_6);

        angularRes = new QSpinBox(widget_2);
        angularRes.setObjectName("angularRes");

        horizontalLayout_5.addWidget(angularRes);


        verticalLayout_2.addWidget(widget_2);


        horizontalLayout_4.addWidget(widget);

        feeds.addWidget(sonarCamPage);

        horizontalLayout.addWidget(feeds);

        retranslateUi(CameraFeeds);

        feeds.setCurrentIndex(2);


        CameraFeeds.connectSlotsByName();
    } // setupUi

    void retranslateUi(QWidget CameraFeeds)
    {
        CameraFeeds.setWindowTitle(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "Form", null));
        label.setText(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "Direction", null));
        label_2.setText(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "Width", null));
        label_3.setText(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "Gain", null));
        label_4.setText(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "Range", null));
        label_5.setText(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "RadialRes", null));
        label_6.setText(com.trolltech.qt.core.QCoreApplication.translate("CameraFeeds", "AngularRes", null));
    } // retranslateUi

}

