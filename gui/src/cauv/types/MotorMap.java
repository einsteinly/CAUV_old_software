package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class MotorMap {
    public int zeroPlus;
    public int zeroMinus;
    public int maxPlus;
    public int maxMinus;

    public MotorMap()
    {
    }

    public static MotorMap readFrom(LEDataInputStream s) throws IOException {
        MotorMap val = new MotorMap();
        val.zeroPlus = s.readInt();
        val.zeroMinus = s.readInt();
        val.maxPlus = s.readInt();
        val.maxMinus = s.readInt();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeInt(this.zeroPlus);
        s.writeInt(this.zeroMinus);
        s.writeInt(this.maxPlus);
        s.writeInt(this.maxMinus);
    }
}
