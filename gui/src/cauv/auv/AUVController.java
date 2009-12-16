package cauv.auv;

import java.io.IOException;
import java.net.UnknownHostException;

class AUVController implements MessageObserver {

    protected AUV auv;
    protected MessageSocket messages;

    public AUVController(AUV auv, String address, int port) throws UnknownHostException,
            IOException {

        this.auv = auv;

        messages = new MessageSocket(address, port);
        messages.addObserver(this);
        messages.joinGroup("control");
        messages.joinGroup("images");
        messages.joinGroup("trace");

        enable();
    }

    public void enable() {
        auv.motors.HBOW.speedChanged.connect(this, "updateHBow(int)");
        auv.motors.VBOW.speedChanged.connect(this, "updateVBow(int)");
        auv.motors.HSTERN.speedChanged.connect(this, "updateHStern(int)");
        auv.motors.VSTERN.speedChanged.connect(this, "updateVStern(int)");
        auv.motors.PROP.speedChanged.connect(this, "updateProp(int)");
    }

    public void disable() {
        auv.motors.HBOW.speedChanged.disconnect(this, "updateHBow(int)");
        auv.motors.VBOW.speedChanged.disconnect(this, "updateVBow(int)");
        auv.motors.HSTERN.speedChanged.disconnect(this, "updateHStern(int)");
        auv.motors.VSTERN.speedChanged.disconnect(this, "updateVStern(int)");
        auv.motors.PROP.speedChanged.disconnect(this, "updateProp(int)");
    }

    protected void updateHBow(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.HBOW, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating HBOW: " + e.getMessage());
        }
    }

    protected void updateVBow(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.VBOW, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating VBOW: " + e.getMessage());
        }
    }

    protected void updateHStern(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.HSTERN, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating HSTERN: " + e.getMessage());
        }
    }

    protected void updateVStern(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.VSTERN, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating VSTERN: " + e.getMessage());
        }
    }

    protected void updateProp(int speed) {
        try {
            messages.sendMessage(new MotorMessage(MotorID.PROP, (byte) speed));
        } catch (IOException e) {
            auv.logs.ERROR.log("Error updating PROP: " + e.getMessage());
        }
    }

    @Override
    public void onImageMessage(ImageMessage m) {
        switch (m.m_source) {
            case FORWARD:
                auv.cameras.FORWARD.updateImage(m.m_image);
                break;
            case DOWN:
                auv.cameras.DOWNWARD.updateImage(m.m_image);
                break;
            case SONAR:
                auv.cameras.SONAR.updateImage(m.m_image);
                break;
        }
    }

    @Override
    public void onMotorMessage(MotorMessage m) {
        switch (m.m_motorId) {
            case PROP:
                auv.motors.PROP.updateSpeed(m.m_speed);
                break;
            case HBOW:
                auv.motors.HBOW.updateSpeed(m.m_speed);
                break;
            case HSTERN:
                auv.motors.HSTERN.updateSpeed(m.m_speed);
                break;
            case VBOW:
                auv.motors.VBOW.updateSpeed(m.m_speed);
                break;
            case VSTERN:
                auv.motors.VSTERN.updateSpeed(m.m_speed);
                break;
        }
    }

    @Override
    public void onTelemetryMessage(TelemetryMessage m) {
        auv.setOrientation(m.m_orientation);
    }
}
