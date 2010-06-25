package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class floatXYZ {
    public float x;
    public float y;
    public float z;

    public floatXYZ()
    {
    }

    public static floatXYZ readFrom(DataOutputStream s) throws IOException {
        floatXYZ val = new floatXYZ();
        s.writeFloat(val.x);
        s.writeFloat(val.y);
        s.writeFloat(val.z);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.x = s.readFloat();
        this.y = s.readFloat();
        this.z = s.readFloat();
    }
}
