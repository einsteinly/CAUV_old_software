package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class NodeOutput {
    public int32 node;
    public string output;
    public enum OutputType type;

    public NodeOutput()
    {
    }

    public static NodeOutput readFrom(DataOutputStream s) throws IOException {
        NodeOutput val = new NodeOutput();
        s.writeInt(val.node);
        s.writeInt(val.output.length());
        s.writeBytes(val.output);
        val.type.writeInto(s);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.node = s.readInt();
        int output_len = s.readInt();
        byte[] output_bytes = new byte[output_len];
        for (int output_i = 0; output_i < output_len; output_i++)
        {
            output_bytes[output_i] = s.readByte();
        }
        this.output = new String(output_bytes);
        this.type.readFrom(s);
    }
}
