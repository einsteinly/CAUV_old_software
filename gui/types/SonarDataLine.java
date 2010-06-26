package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class SonarDataLine {
    public list< byte > data;
    public int32 bearing;
    public int32 bearingRange;
    public int32 range;

    public SonarDataLine()
    {
    }

    public static SonarDataLine readFrom(DataOutputStream s) throws IOException {
        SonarDataLine val = new SonarDataLine();
        s.writeLong(val.data.size());
        for (int data_i = 0; data_i < val.data.size(); data_i++)
        {
            byte data_val = val.data.get(data_i);
            s.writeByte(data_val);
        }
        s.writeInt(val.bearing);
        s.writeInt(val.bearingRange);
        s.writeInt(val.range);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.data = new Vector< Byte >();
        long $(len)s = s.readLong();
        for (int data_i = 0; data_i < data_len; data_i++)
        {
            byte data_val;
            data_val = s.readByte();
            this.data.add(data_i, data_val);
        }
        this.bearing = s.readInt();
        this.bearingRange = s.readInt();
        this.range = s.readInt();
    }
}
