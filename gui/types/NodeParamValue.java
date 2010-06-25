package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class NodeParamValue {
    public enum ParamType type;
    public int32 intValue;
    public float floatValue;
    public string stringValue;

    public NodeParamValue()
    {
    }

    public static NodeParamValue readFrom(DataOutputStream s) throws IOException {
        NodeParamValue val = new NodeParamValue();
        val.type.writeInto(s);
        s.writeInt(val.intValue);
        s.writeFloat(val.floatValue);
        s.writeInt(val.stringValue.length());
        s.writeBytes(val.stringValue);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.type.readFrom(s);
        this.intValue = s.readInt();
        this.floatValue = s.readFloat();
        int stringValue_len = s.readInt();
        byte[] stringValue_bytes = new byte[stringValue_len];
        for (int stringValue_i = 0; stringValue_i < stringValue_len; stringValue_i++)
        {
            stringValue_bytes[stringValue_i] = s.readByte();
        }
        this.stringValue = new String(stringValue_bytes);
    }
}
