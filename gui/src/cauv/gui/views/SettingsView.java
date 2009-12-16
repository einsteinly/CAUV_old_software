package cauv.gui.views;

import cauv.Config;
import cauv.gui.ScreenView;

import com.trolltech.qt.gui.*;

public class SettingsView extends QWidget implements ScreenView {

	public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    Ui_SettingsView ui = new Ui_SettingsView();
	
    public SettingsView() {
        ui.setupUi(this);
		icon.setPixmap(new QPixmap("classpath:cauv/gui/resources/settings.png"));

		load();
		
		ui.saveSettingsButton.clicked.connect(this, "save()");
		ui.resetSettingsButton.clicked.connect(this, "load()");
    }

    public void save(){
    	Config.GAMEPAD_ID = ui.gamepadID.value();
    }
    
    public void load(){
		ui.gamepadID.setValue(Config.GAMEPAD_ID);
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
		return "Settings";
	}
}
