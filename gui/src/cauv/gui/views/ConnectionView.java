package cauv.gui.views;

import java.io.IOException;

import cauv.Config;
import cauv.auv.AUV;
import cauv.auv.MessageSocket;
import cauv.auv.MessageSocket.ConnectionStateObserver;
import cauv.gui.ScreenView;

import com.trolltech.qt.gui.*;

public class ConnectionView extends QWidget implements ScreenView, ConnectionStateObserver {

    public QGraphicsPixmapItem icon = new QGraphicsPixmapItem();
    Ui_ConnectionView ui = new Ui_ConnectionView();
    
    public ConnectionView() {
        ui.setupUi(this);
        icon.setPixmap(new QPixmap("classpath:cauv/gui/resources/logo.png"));
        ui.address.setText(Config.ADDRESS);
        ui.port.setValue(Config.AUV_PORT);
        ui.connectButton.clicked.connect(this, "connect()");
    }
    
    protected void connect() {
        ui.address.setEnabled(false);
        ui.port.setEnabled(false);
        ui.connectButton.setEnabled(false);
        ui.errorMessage.setText("Connecting...");
        this.repaint();

        Config.ADDRESS = ui.address.text();
        Config.AUV_PORT = ui.port.value();

        try {
            AUV auv = new AUV(Config.ADDRESS, Config.AUV_PORT);
            auv.regsiterConnectionStateObserver(this);
        } catch (IOException e) {
            ui.errorMessage.setText("Connecting to AUV failed.");
        } finally {
            ui.address.setEnabled(true);
            ui.port.setEnabled(true);
            ui.connectButton.setEnabled(true);
            ui.errorMessage.setText("");
        }

    }
    
    @Override
    public void onConnect(MessageSocket connection) {
        //ui.mainStack.setCurrentWidget(ui.informationPage);
    }

    @Override
    public void onDisconnect(MessageSocket connection) {
        //ui.mainStack.setCurrentWidget(ui.addressPage);
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
        return "Connection";
    }
}
