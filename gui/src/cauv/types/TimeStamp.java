package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class TimeStamp {
    public int secs;
    public int msecs;

    public TimeStamp()
    {
    }

    public static TimeStamp readFrom(LEDataInputStream s) throws IOException {
        TimeStamp val = new TimeStamp();
        val.secs = s.readInt();
        val.msecs = s.readInt();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeInt(this.secs);
        s.writeInt(this.msecs);
    }
}
