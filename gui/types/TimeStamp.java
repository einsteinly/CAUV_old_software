package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class TimeStamp {
    public int32 secs;
    public int32 msecs;

    public TimeStamp()
    {
    }

    public static TimeStamp readFrom(DataOutputStream s) throws IOException {
        TimeStamp val = new TimeStamp();
        s.writeInt(val.secs);
        s.writeInt(val.msecs);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.secs = s.readInt();
        this.msecs = s.readInt();
    }
}
