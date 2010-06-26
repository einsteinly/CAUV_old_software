package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class NodeOutputArc {
    public struct NodeInput dst;
    public string output;

    public NodeOutputArc()
    {
    }

    public static NodeOutputArc readFrom(DataOutputStream s) throws IOException {
        NodeOutputArc val = new NodeOutputArc();
        val.dst.writeInto(s);
        s.writeInt(val.output.length());
        s.writeBytes(val.output);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.dst.readFrom(s);
        int output_len = s.readInt();
        byte[] output_bytes = new byte[output_len];
        for (int output_i = 0; output_i < output_len; output_i++)
        {
            output_bytes[output_i] = s.readByte();
        }
        this.output = new String(output_bytes);
    }
}
