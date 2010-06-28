package cauv.gamepad;

import java.io.IOException;
import java.util.Vector;

import net.java.games.input.*;

/**
 * Singleton class that allows input from a connected PS2 controller.
 * Objects can subscribe to the controller's events by implementing
 * Ps2InputHandler and then calling 'subscribe'.
 *
 * Will generate a null pointer exception if an object subscribes and then
 * gets garbage-collected.
 * @author Alex Nixon
 */
public class PS2Controller
{

    public enum Button
    {

        TRIANGLE,
        CIRCLE,
        X,
        SQUARE,
        L2,
        R2,
        L1,
        R1,
        SELECT,
        START,
        L3,
        R3,
        LEFT,
        RIGHT,
        UP,
        DOWN,
        JOY_L_X,
        JOY_L_Y,
        JOY_R_X,
        JOY_R_Y,
        JOY_L,
        JOY_R,
        UNKNOWN
    }
    private final Vector<PS2InputHandler> observers;

    /**
     * Creates a new instance of ps2Controller
     */
    public PS2Controller(int controllerID) throws IOException
    {
        observers = new Vector<PS2InputHandler>();

        Controller cont = null;

        Controller[] ca =
            ControllerEnvironment.getDefaultEnvironment().getControllers();
        if (ca.length == 0) {
            throw new IOException("No gamepad controllers found");
        }
        // Try find the PS2 controller...

        try {
            cont = ca[controllerID];
        }
        catch (ArrayIndexOutOfBoundsException ex) {
            throw new IOException("No such gamepad with ID "+ controllerID);
        }

        new PS2Reader(cont, this);
    }

    public void onEvent(Event e)
    {
        Component comp = e.getComponent();
        Button btn = null;

        // keypad
        if (comp.getIdentifier().toString().equals("pov") && comp.getPollData() == 0.25)
            btn = Button.UP;
        else if (comp.getIdentifier().toString().equals("pov") && comp.getPollData() == 0.5)
            btn = Button.RIGHT;
        else if (comp.getIdentifier().toString().equals("pov") && comp.getPollData() == 0.75)
            btn = Button.DOWN;
        else if (comp.getIdentifier().toString().equals("pov") && comp.getPollData() == 1.0)
            btn = Button.LEFT;
        //joysticks
        else if (comp.getIdentifier().toString().equals("x"))
            btn = Button.JOY_L_X;
        else if (comp.getIdentifier().toString().equals("y"))
            btn = Button.JOY_L_Y;
        else if (comp.getIdentifier().toString().equals("rz"))
            btn = Button.JOY_R_X;
        else if (comp.getIdentifier().toString().equals("z"))
            btn = Button.JOY_R_Y;
        //circle buttons
        else if (comp.getIdentifier().toString().equals("0") ||
                comp.getIdentifier().toString().equals("Trigger"))
            btn = Button.TRIANGLE;
        else if (comp.getIdentifier().toString().equals("1") ||
                comp.getIdentifier().toString().equals("Thumb"))
            btn = Button.CIRCLE;
        else if (comp.getIdentifier().toString().equals("2") ||
                comp.getIdentifier().toString().equals("Thumb 2"))
            btn = Button.X;
        else if (comp.getIdentifier().toString().equals("3") ||
                comp.getIdentifier().toString().equals("Top"))
            btn = Button.SQUARE;
        //top buttons
        else if (comp.getIdentifier().toString().equals("4") ||
                comp.getIdentifier().toString().equals("Top 2"))
            btn = Button.L2;
        else if (comp.getIdentifier().toString().equals("5") ||
                comp.getIdentifier().toString().equals("Pinkie"))
            btn = Button.R2;
        else if (comp.getIdentifier().toString().equals("6") ||
                comp.getIdentifier().toString().equals("Base"))
            btn = Button.L1;
        else if (comp.getIdentifier().toString().equals("7") ||
                comp.getIdentifier().toString().equals("Base 2"))
            btn = Button.R1;
        // click joy sticks
        else if (comp.getIdentifier().toString().equals("10") ||
                comp.getIdentifier().toString().equals("Base 5"))
            btn = Button.JOY_L;
        else if (comp.getIdentifier().toString().equals("11") ||
                comp.getIdentifier().toString().equals("Base 6"))
            btn = Button.JOY_R;



        // tell our observers
        if (btn != null)
            for (PS2InputHandler observer : observers)
                observer.onPressed(btn, comp.getPollData());

    }


    /**
     *  Subscribe to receive notifications of button presses.  The methods
     *  onButtonDown(Button b) and onButtonUp(Button b) should be handled using
     *  a switch statement on the argument.
     * @param listener 
     */
    public void registerObserver(final PS2InputHandler listener)
    {
        observers.add(listener);
    }

    public static String listControllers()
    {
        String ret = new String();
        
        Controller[] ca =
            ControllerEnvironment.getDefaultEnvironment().getControllers();
        int count = 0;
        for (Controller c : ca) {
            ret += (count++ + " " + c.getName() + "\n");
        }
        
        return ret;
    }
}

