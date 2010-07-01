package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class SonarDataLine {
    public Vector< Byte > data;
    public int bearing;
    public int bearingRange;
    public int range;

    public SonarDataLine()
    {
    }

    public static SonarDataLine readFrom(LEDataInputStream s) throws IOException {
        SonarDataLine val = new SonarDataLine();
        val.data = new Vector< Byte >();
        long data_len = s.readInt();
        for (int data_i = 0; data_i < data_len; data_i++)
        {
            byte data_val;
            data_val = s.readByte();
            val.data.add(data_i, data_val);
        }
        val.bearing = s.readInt();
        val.bearingRange = s.readInt();
        val.range = s.readInt();
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeInt(this.data.size());
        for (int data_i = 0; data_i < this.data.size(); data_i++)
        {
            Byte data_val = this.data.get(data_i);
            s.writeByte(data_val);
        }
        s.writeInt(this.bearing);
        s.writeInt(this.bearingRange);
        s.writeInt(this.range);
    }
}
