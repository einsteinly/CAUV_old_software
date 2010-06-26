package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class floatYPR {
    public float yaw;
    public float pitch;
    public float roll;

    public floatYPR()
    {
    }

    public static floatYPR readFrom(DataOutputStream s) throws IOException {
        floatYPR val = new floatYPR();
        s.writeFloat(val.yaw);
        s.writeFloat(val.pitch);
        s.writeFloat(val.roll);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.yaw = s.readFloat();
        this.pitch = s.readFloat();
        this.roll = s.readFloat();
    }
}
