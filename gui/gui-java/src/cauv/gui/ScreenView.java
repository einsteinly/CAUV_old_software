package cauv.gui;

import cauv.auv.AUV;
import cauv.auv.CommunicationController.AUVConnectionObserver;

import com.trolltech.qt.gui.QGraphicsItemInterface;
import com.trolltech.qt.gui.QWidget;

public interface ScreenView extends AUVConnectionObserver {

	public QGraphicsItemInterface getIconWidget();
	
	public QWidget getScreenWidget();
	
	public String getScreenName();

    public void onConnect(AUV auv);
    
    public void onDisconnect();
}
