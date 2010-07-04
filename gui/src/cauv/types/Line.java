package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class Line {
    public floatXYZ centre;
    public float angle;

    public Line()
    {
    }

    public static Line readFrom(LEDataInputStream s) throws IOException {
        Line val = new Line();
        val.centre = floatXYZ.readFrom(s);
        val.angle = s.readFloat();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        this.centre.writeInto(s);
        s.writeFloat(this.angle);
    }
}
