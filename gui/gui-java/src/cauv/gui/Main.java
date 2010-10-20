package cauv.gui;

import java.io.IOException;
import java.util.Vector;

import spread.SpreadGroup;
import spread.SpreadMessage;

import cauv.Config;
import cauv.auv.AUV;
import cauv.auv.MessageSocket;
import cauv.auv.MessageSocket.ConnectionStateObserver;
import cauv.auv.MessageSocket.MembershipObserver;
import cauv.gui.controllers.PS2ControlHandler;
import cauv.gui.dialogs.Settings;
import cauv.gui.views.CameraFeeds;
import cauv.gui.views.MissionControlView;
import cauv.gui.views.MotorControlView;
import cauv.gui.views.TelemetryView;

import com.trolltech.qt.core.QEvent;
import com.trolltech.qt.core.QObject;
import com.trolltech.qt.core.QTimer;
import com.trolltech.qt.core.Qt;
import com.trolltech.qt.core.QEvent.Type;
import com.trolltech.qt.core.Qt.ConnectionType;
import com.trolltech.qt.core.Qt.Key;
import com.trolltech.qt.core.Qt.KeyboardModifier;
import com.trolltech.qt.gui.*;

public class Main extends QMainWindow implements ConnectionStateObserver, MembershipObserver {

	Ui_Main ui = new Ui_Main();
	
	static Main auvGUI;
	Settings settingsDialog = new Settings();
	
	AUV auv;
	Vector<ScreenView> views = new Vector<ScreenView>();
	Vector<String> members = new Vector<String>();
	Vector<String> commandHistory = new Vector<String>();
	String last;
	int history = 0;
	
	public static void main(String[] args) {
		QApplication.initialize(args);
		try {
			Config.load();
		} catch (IOException e) {
			System.err.println("Config error: " + e.getMessage());
		}

		Main gui = new Main();
		
		gui.registerScreen(new CameraFeeds());
		gui.registerScreen(new TelemetryView());
		gui.registerScreen(new MissionControlView());
		gui.registerScreen(new MotorControlView());
		
		gui.show();

		QApplication.exec();

		try {
			Config.save();
		} catch (IOException e) {
			System.err.println("Config error: " + e.getMessage());
		}
	}

	public Main() {
		auvGUI = this;
		ui.setupUi(this);
		ui.address.setText(Config.ADDRESS);
		ui.port.setValue(Config.AUV_PORT);
		//ui.backButton.hide();
		//ui.backButton.clicked.connect(this, "back()");
		ui.connectButton.clicked.connect(this, "connect()");
		ui.actionOptions_2.triggered.connect(this, "showSettings()");
		AUV.registerAUVConnectionObserver(settingsDialog);
        ui.pythonConsole.returnPressed.connect(this, "sendPython()");
		
		Main.trace("Initialisation complete");
		
		QTimer t = new QTimer(this);
		t.setInterval(200);
		t.setSingleShot(false);
		t.timeout.connect(this, "updateMembershipLights()");
		t.start();
		

        ui.pythonConsole.installEventFilter(this);
	}


	public boolean eventFilter(QObject obj, QEvent event) {
	    
	    if (obj == ui.pythonConsole)
	    {
	        if (event.type() == Type.KeyPress)
	        {
	            QKeyEvent keyEvent = (QKeyEvent)event;
	            if (keyEvent.key() == Key.Key_Up.value())
	            {
	                 historyUp();
	                 return true;
	            }
	            else if(keyEvent.key() == Key.Key_Down.value())
	            {
	                historyDown();
	                return true;
	            }
	        }
	        return super.eventFilter(obj, event);
	    }
	    return super.eventFilter(obj, event);
	}
	
	public void showSettings(){
	    settingsDialog.show();
	}
	
	protected void back(){
		ui.informationStack.setCurrentWidget(ui.page);
		//ui.backButton.hide();
	}
	
	protected void connect() {
	    AUV.getAllStreams().clear();
	    
		ui.address.setEnabled(false);
		ui.port.setEnabled(false);
		ui.connectButton.setEnabled(false);
		ui.errorMessage.setText("Connecting...");
		this.repaint();

		Config.ADDRESS = ui.address.text();
		Config.AUV_PORT = ui.port.value();

		try {
			this.auv = new AUV(Config.ADDRESS, Config.AUV_PORT);
			auv.regsiterConnectionStateObserver(this);

	        ui.emergencyStop.clicked.connect(auv, "stopAllMotors()");
			
            auv.logs.TRACE.messageLogged.connect(this, "trace(String)", ConnectionType.BlockingQueuedConnection);
            auv.logs.DEBUG.messageLogged.connect(this, "warning(String)", ConnectionType.BlockingQueuedConnection);
            auv.logs.ERROR.messageLogged.connect(this, "error(String)", ConnectionType.BlockingQueuedConnection);
            
            auv.scripting.CONSOLE.responseReceieved.connect(ui.pythonDisplay, "append(String)");
            
			new PS2ControlHandler(auv);
		} catch (IOException e) {
			ui.errorMessage.setText("Connecting to AUV failed. Sigh.");
			ui.address.setEnabled(true);
			ui.port.setEnabled(true);
		} finally {
			ui.connectButton.setEnabled(true);
		}
	}
	
	public void registerScreen(final ScreenView view) {
		
	    AUV.registerAUVConnectionObserver(view);
	    
		Main.trace("Registering view ["+view.getScreenName()+"]");
		views.add(view);
		
		// add the main screen to the stack of information panels
		ui.informationStack.addWidget(view.getScreenWidget());
		
		// set up a mouse call-back on the icon so that when it's clicked
		// the main screen is displayed by moving it to the top of the
		// stack
		QGraphicsView graphics = new QGraphicsView() {
			protected void mouseReleaseEvent(QMouseEvent arg) {
				//ui.backButton.show();
				ui.informationStack.setCurrentWidget(view.getScreenWidget());
			}
		};		
		
		graphics.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff);
		graphics.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff);

		ScreenIcon icon = new ScreenIcon();
		icon.addWidget(graphics);
		icon.setText(view.getScreenName());
		ui.iconLayout.addWidget(icon);
		
		QGraphicsScene scene = new QGraphicsScene();
		scene.addItem(view.getIconWidget());
		graphics.setScene(scene);
	};

	public static void message(String message){
		auvGUI.ui.all.append(message);
	}
	
	public static void error(String message){
		auvGUI.ui.errors.append(message);
		String icon = "<img style='vertical-align:baseline' src=\"classpath:cauv/gui/resources/icons/sign_remove.png\" /> ";
		message = "<span style=\" color:#8d162c;\">"+message+"</span>";
		Main.message(icon + message);
	}
	
	public static void warning(String message){
		auvGUI.ui.warnings.append(message);
		String icon = "<img style='vertical-align:baseline' src=\"classpath:cauv/gui/resources/icons/sign_warning.png\" /> ";
		message = "<span style=\" color:#8d5f14;\">"+message+"</span>";
		Main.message(icon + message);
	}
	
	public static void trace(String message){
		auvGUI.ui.traces.append(message);
		String icon = "<img style='vertical-align:baseline' src=\"classpath:cauv/gui/resources/icons/sign_info.png\" /> ";
		message = "<span style=\" color:#005632;\">"+message+"</span>";
		Main.message(icon + message);
	}
	
	public void updateHistoryEdit(){
	    if(history == 0) return;
	    if(!commandHistory.isEmpty() && commandHistory.size() >= history )
	    ui.pythonConsole.setText(commandHistory.get(commandHistory.size() - history));
	}
	
	public void historyUp(){
	    if(this.history == 0){
	        this.last = ui.pythonConsole.text();
	    }
	    
	    if(history < commandHistory.size())
	        history++;
	    updateHistoryEdit();	   
	}
	    
	public void historyDown(){
	    if(history > 0)
	        history--;
	    updateHistoryEdit();
	        
	    if(this.history == 0){
	        ui.pythonConsole.setText(this.last);
	    }	    
	}
	
	public void sendPython(){
	    if(auv != null) {
	        ui.pythonDisplay.append(ui.pythonConsole.text());
	        auv.scripting.CONSOLE.run(ui.pythonConsole.text());
	        commandHistory.add(ui.pythonConsole.text());
	        history = 0;
	        ui.pythonConsole.clear();
	    } else {
	        ui.pythonDisplay.append("AUV not connected");
	    }
	}
	
	@Override
	public void onConnect(MessageSocket connection) {
        connection.addMembershipObserver(this);
		ui.connectButton.clicked.disconnect(this, "connect()");
		ui.connectButton.clicked.connect(connection, "disconnect()");
		ui.address.setEnabled(false);
		ui.port.setEnabled(false);
		ui.connectButton.setText("Disconnect");
		ui.errorMessage.setText("");
	}

	@Override
	public void onDisconnect(MessageSocket connection) {
		ui.address.setEnabled(true);
		ui.port.setEnabled(true);
		ui.connectButton.setText("Connect");

		members.clear();
		updateMembershipLights();
		
        ui.connectButton.clicked.connect(this, "connect()");
        ui.connectButton.clicked.disconnect(connection, "disconnect()");
	}

	@Override
	protected void keyPressEvent(QKeyEvent arg) {
		if (arg.key() == Key.Key_Escape.value()) {
			if(this.isFullScreen())
				this.showNormal();
			else this.back();
		} else if ((arg.modifiers().isSet(KeyboardModifier.AltModifier))
				&& (arg.key() == Key.Key_Return.value())) {
			this.showFullScreen();
		}
		super.keyPressEvent(arg);
	}

	
	public void updateMembershipLights(){
	    
        ui.controlLED.setText("<img src=\"classpath:cauv/gui/resources/red-led.png\" />");
        ui.aiLED.setText("<img src=\"classpath:cauv/gui/resources/red-led.png\" />");
        ui.imageProcLED.setText("<img src=\"classpath:cauv/gui/resources/red-led.png\" />");
        ui.pipeGUILED.setText("<img src=\"classpath:cauv/gui/resources/red-led.png\" />");
        
        for(String member : members){
            if(member.toLowerCase().equals("control")){
                ui.controlLED.setText("<img src=\"classpath:cauv/gui/resources/green-led.png\" />");
            }
            if(member.toLowerCase().equals("ai")){
                ui.aiLED.setText("<img src=\"classpath:cauv/gui/resources/green-led.png\" />");
            }
            if(member.toLowerCase().equals("img-pipe")){
                ui.imageProcLED.setText("<img src=\"classpath:cauv/gui/resources/green-led.png\" />");
            }
            if(member.toLowerCase().equals("pipe-gui")){
                ui.pipeGUILED.setText("<img src=\"classpath:cauv/gui/resources/green-led.png\" />");
            }
        }
	}
	
	
    @Override
    public void onMembershipChanged(SpreadMessage message) {
        
        for(SpreadGroup g: message.getMembershipInfo().getMembers()){
            String member = g.toString().substring(1, g.toString().indexOf("#", 2));
            if(!members.contains(member))
            {
                members.add(member);
                auv.logs.TRACE.log(member + " connected");
            }
        }
        
        if(message.getMembershipInfo().isCausedByLeave()){
            SpreadGroup g = message.getMembershipInfo().getLeft();
            String member = g.toString().substring(1, g.toString().indexOf("#", 2));
            if(members.contains(member)){
                members.remove(member);
                auv.logs.TRACE.log(member + " left");
            }
        }
        else if(message.getMembershipInfo().isCausedByDisconnect())
        {
            SpreadGroup g = message.getMembershipInfo().getDisconnected();
            String member = g.toString().substring(1, g.toString().indexOf("#", 2));
            if(members.contains(member)){
                members.remove(member);
                auv.logs.TRACE.log(member + " disconnected");
            }
        }
    }
}
