package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class MotorDemand {
    public float prop;
    public float hbow;
    public float vbow;
    public float hstern;
    public float vstern;

    public MotorDemand()
    {
    }

    public static MotorDemand readFrom(LEDataInputStream s) throws IOException {
        MotorDemand val = new MotorDemand();
        val.prop = s.readFloat();
        val.hbow = s.readFloat();
        val.vbow = s.readFloat();
        val.hstern = s.readFloat();
        val.vstern = s.readFloat();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeFloat(this.prop);
        s.writeFloat(this.hbow);
        s.writeFloat(this.vbow);
        s.writeFloat(this.hstern);
        s.writeFloat(this.vstern);
    }
}
