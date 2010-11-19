package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class NodeParamValue {
    public ParamType type;
    public int intValue;
    public float floatValue;
    public String stringValue;

    public NodeParamValue()
    {
    }

    public static NodeParamValue readFrom(LEDataInputStream s) throws IOException {
        NodeParamValue val = new NodeParamValue();
        val.type = ParamType.readFrom(s);
        val.intValue = s.readInt();
        val.floatValue = s.readFloat();
        int stringValue_len = s.readInt();
        byte[] stringValue_bytes = new byte[stringValue_len];
        for (int stringValue_i = 0; stringValue_i < stringValue_len; stringValue_i++)
        {
            stringValue_bytes[stringValue_i] = s.readByte();
        }
        val.stringValue = new String(stringValue_bytes);
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        this.type.writeInto(s);
        s.writeInt(this.intValue);
        s.writeFloat(this.floatValue);
        s.writeInt(this.stringValue.length());
        s.writeBytes(this.stringValue);
    }
}
