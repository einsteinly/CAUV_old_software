package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class floatYPR {
    public float yaw;
    public float pitch;
    public float roll;

    public floatYPR()
    {
    }

    public static floatYPR readFrom(LEDataInputStream s) throws IOException {
        floatYPR val = new floatYPR();
        val.yaw = s.readFloat();
        val.pitch = s.readFloat();
        val.roll = s.readFloat();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeFloat(this.yaw);
        s.writeFloat(this.pitch);
        s.writeFloat(this.roll);
    }
}
