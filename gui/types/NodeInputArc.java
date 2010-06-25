package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class NodeInputArc {
    public string input;
    public struct NodeOutput src;

    public NodeInputArc()
    {
    }

    public static NodeInputArc readFrom(DataOutputStream s) throws IOException {
        NodeInputArc val = new NodeInputArc();
        s.writeInt(val.input.length());
        s.writeBytes(val.input);
        val.src.writeInto(s);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        int input_len = s.readInt();
        byte[] input_bytes = new byte[input_len];
        for (int input_i = 0; input_i < input_len; input_i++)
        {
            input_bytes[input_i] = s.readByte();
        }
        this.input = new String(input_bytes);
        this.src.readFrom(s);
    }
}
