package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class Line {
    public floatXYZ a;
    public floatXYZ b;

    public Line()
    {
    }

    public static Line readFrom(LEDataInputStream s) throws IOException {
        Line val = new Line();
        val.a = floatXYZ.readFrom(s);
        val.b = floatXYZ.readFrom(s);
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        this.a.writeInto(s);
        this.b.writeInto(s);
    }
}
