package cauv.gui;

import java.io.IOException;

import cauv.auv.AUV;
import cauv.gui.Ui_GUIMain;
import cauv.gui.controllers.ControlHandler;
import cauv.gui.views.CameraFeeds;
import cauv.gui.views.ConnectionView;
import cauv.gui.views.MissionControlView;
import cauv.gui.views.SettingsView;
import cauv.gui.views.TelemetryView;
import cauv.Config;

import com.trolltech.qt.core.Qt;
import com.trolltech.qt.core.Qt.Key;
import com.trolltech.qt.core.Qt.KeyboardModifier;
import com.trolltech.qt.gui.*;

public class GUIMain extends QMainWindow {

	Ui_GUIMain ui = new Ui_GUIMain();
	
	public Signal1<Boolean> controlStateChanged = new Signal1<Boolean>();

	public GUIMain() {
		ui.setupUi(this);
        ui.controlsToggle.clicked.connect(this, "toggleControls()");
        AUV.connection.connect.connect(this, "onConnect(AUV)");
		ui.controls.hide();
	}

	public GUIMain(QWidget parent) {
		super(parent);
		ui.setupUi(this);
	}

	@Override
	protected void keyPressEvent(QKeyEvent arg) {		
		if (arg.key() == Key.Key_Escape.value()) {
			this.showNormal();
		} else if ((arg.modifiers().isSet(KeyboardModifier.AltModifier)) 
				&& (arg.key() == Key.Key_Return.value())){
			this.showFullScreen();
		}
		super.keyPressEvent(arg);
	}

	protected void toggleControls() {
		if (ui.controls.isVisible()) {
			ui.controls.hide();
			ui.controlsToggle.setIcon(new QIcon(new QPixmap(
					"classpath:cauv/gui/resources/arrow.png")));
			controlStateChanged.emit(false);
		} else {
			ui.controls.show();
			ui.controlsToggle.setIcon(new QIcon(new QPixmap(
					"classpath:cauv/gui/resources/arrow_down.png")));
			controlStateChanged.emit(true);
		}
	}

	public void registerScreen(final ScreenView view) {
		// add the main screen to the stack of information panels
		ui.informationStack.addWidget(view.getScreenWidget());

		// set up a mouse call-back on the icon so that when it's clicked
		// the main screen is displayed by moving it to the top of the
		// stack
		
		QGraphicsView graphics = new QGraphicsView() {
			protected void mouseReleaseEvent(QMouseEvent arg) {
			    ui.informationStack.setCurrentWidget(view.getScreenWidget());
			}
		};
		graphics.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff);
		graphics.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff);
		graphics.setFrameStyle(0);
		graphics.setAutoFillBackground(false);
		graphics.setBackgroundBrush(new QBrush(new QColor(27, 27, 27)));
		graphics.setCursor(new QCursor(Qt.CursorShape.PointingHandCursor));

		ScreenIcon icon = new ScreenIcon();
		icon.addWidget(graphics);
		icon.setText(view.getScreenName());
		ui.iconLayout.addWidget(icon);
		
		QGraphicsScene scene = new QGraphicsScene();
		scene.addItem(view.getIconWidget());
		scene.setBackgroundBrush(new QBrush(QColor.transparent));
		graphics.setScene(scene);
	};
	
	
	public static void main(String[] args) {
		QApplication.initialize(args);

		try {
			Config.load();
		} catch (IOException e) {
			System.err.println("Config error: " + e.getMessage());
		}

		GUIMain gui = new GUIMain();
        gui.registerScreen(new ConnectionView());
        gui.registerScreen(new CameraFeeds());
		gui.registerScreen(new TelemetryView());
		gui.registerScreen(new MissionControlView());
		gui.registerScreen(new SettingsView());
		gui.show();
		QApplication.exec();

		try {
			Config.save();
		} catch (IOException e) {
			System.err.println("Config error: " + e.getMessage());
		}
	}

    public void onConnect(AUV auv) {
        new ControlHandler(this.ui, auv);
        ui.informationStack.setCurrentIndex(4);
    }

}
