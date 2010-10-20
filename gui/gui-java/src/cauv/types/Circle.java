package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class Circle {
    public floatXYZ centre;
    public float radius;

    public Circle()
    {
    }

    public static Circle readFrom(LEDataInputStream s) throws IOException {
        Circle val = new Circle();
        val.centre = floatXYZ.readFrom(s);
        val.radius = s.readFloat();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        this.centre.writeInto(s);
        s.writeFloat(this.radius);
    }
}
