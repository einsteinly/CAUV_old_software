package cauv.gamepad;


/**
 * To be implemented by any classes which wish to be notified when a button is
 * pressed on the PS2 controller.
 * @author Andy Pritchard
 */
public interface PS2InputHandler {

    public void onPressed (PS2Controller.Button button, float value);

}
