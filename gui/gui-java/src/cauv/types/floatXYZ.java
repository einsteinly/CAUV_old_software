package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class floatXYZ {
    public float x;
    public float y;
    public float z;

    public floatXYZ()
    {
    }

    public static floatXYZ readFrom(LEDataInputStream s) throws IOException {
        floatXYZ val = new floatXYZ();
        val.x = s.readFloat();
        val.y = s.readFloat();
        val.z = s.readFloat();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeFloat(this.x);
        s.writeFloat(this.y);
        s.writeFloat(this.z);
    }
}
