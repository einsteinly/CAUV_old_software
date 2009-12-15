package cauv.gui.controllers;

import java.io.IOException;
import cauv.Config;
import cauv.auv.AUV;
import cauv.gamepad.PS2Controller;
import cauv.gamepad.PS2Controller.Button;
import cauv.gui.Ui_GUIMain;

public class ControlHandler implements cauv.gamepad.PS2InputHandler {

    public Ui_GUIMain ui;
    AUV auv;
    protected boolean ignoreInput = true;

    public ControlHandler(Ui_GUIMain ui, AUV auv) {
        this.ui = ui;
        this.auv = auv;

        try {
            System.out.println(PS2Controller.listControllers());
            PS2Controller ps2 = new PS2Controller(Config.GAMEPAD_ID);
            ps2.registerObserver(this);
        } catch (IOException e) {
            System.err.println("Gamepad not availible: " + e.getMessage());
        }

        ui.controlsToggle.clicked.connect(this, "toggleInput()");

        ui.shutdownButton.released.connect(this, "stop()");
        
        ui.sliderHBow.sliderMoved.connect(auv.motors.HBOW, "setSpeed(int)");
        ui.sliderVBow.sliderMoved.connect(auv.motors.VBOW, "setSpeed(int)");
        ui.sliderHStern.sliderMoved.connect(auv.motors.HSTERN, "setSpeed(int)");
        ui.sliderVStern.sliderMoved.connect(auv.motors.VSTERN, "setSpeed(int)");
        ui.sliderProp.sliderMoved.connect(auv.motors.PROP, "setSpeed(int)");

        auv.motors.HBOW.speedChanged.connect(ui.sliderHBow, "setValue(int)");
        auv.motors.VBOW.speedChanged.connect(ui.sliderVBow, "setValue(int)");
        auv.motors.HSTERN.speedChanged.connect(ui.sliderHStern, "setValue(int)");
        auv.motors.VSTERN.speedChanged.connect(ui.sliderVStern, "setValue(int)");
        auv.motors.PROP.speedChanged.connect(ui.sliderProp, "setValue(int)");

    }

    public void toggleInput() {
        this.ignoreInput = !ignoreInput;
    }

    public void stop() {
        auv.motors.HBOW.setSpeed(0);
        auv.motors.HSTERN.setSpeed(0);
        auv.motors.PROP.setSpeed(0);
        auv.motors.VBOW.setSpeed(0);
        auv.motors.VSTERN.setSpeed(0);

        System.out.println("Stopped");
    }

    @Override
    public void onPressed(final Button button, final float value) {

        if (ignoreInput) return;
        switch (button) {
            case JOY_L_X:
                auv.motors.HBOW.setSpeed((int) ((value * value * value) * 128));
                auv.motors.HSTERN.setSpeed((int) ((value * value * value) * 128));
                break;
            case JOY_L_Y:
                auv.motors.VBOW.setSpeed((int) (-(value * value * value) * 128));
                auv.motors.VSTERN.setSpeed((int) (-(value * value * value) * 128));
                break;
            case JOY_R_X:
                auv.motors.HBOW.setSpeed((int) ((value * value * value) * 128));
                auv.motors.HSTERN.setSpeed((int) (-(value * value * value) * 128));
                break;
            case JOY_R_Y:
                auv.motors.VBOW.setSpeed((int) ((value * value * value) * 128));
                auv.motors.VSTERN.setSpeed((int) (-(value * value * value) * 128));
                break;

            case JOY_L:
                break;
            case JOY_R:
                break;
            case X:
                break;
            case R1:
                auv.motors.PROP.setSpeed(auv.motors.PROP.getSpeed() + 16);
                break;
            case R2:
                auv.motors.PROP.setSpeed(auv.motors.PROP.getSpeed() - 16);
                break;

            case UP:
                break;
            case DOWN:
                break;
            case LEFT:
                break;
            case RIGHT:
                break;

            case TRIANGLE:
                break;
            case SQUARE:
                stop();
                break;
            case CIRCLE:
                break;
        }
    }
}
