package cauv.gui;

import com.trolltech.qt.gui.QGraphicsItemInterface;
import com.trolltech.qt.gui.QWidget;

public interface ScreenView {

	public QGraphicsItemInterface getIconWidget();
	
	public QWidget getScreenWidget();
	
	public String getScreenName();
	
}
