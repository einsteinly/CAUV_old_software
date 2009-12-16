package cauv.gui.views;

import cauv.gui.ScreenView;
import cauv.gui.views.Ui_CameraFeeds;
import com.trolltech.qt.gui.*;


public class CameraFeeds extends QWidget implements ScreenView {

	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    Ui_CameraFeeds ui = new Ui_CameraFeeds();
	
    public CameraFeeds() {
        ui.setupUi(this);
    }

	@Override
	public QGraphicsItemInterface getIconWidget() {
		return icon;
	}

	@Override
	public QWidget getScreenWidget() {
		return this;
	}

	@Override
	public String getScreenName() {
		return "Camera Feeds";
	}
}
