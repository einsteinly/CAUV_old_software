package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class NodeOutput {
    public int node;
    public String output;
    public OutputType type;

    public NodeOutput()
    {
    }

    public static NodeOutput readFrom(LEDataInputStream s) throws IOException {
        NodeOutput val = new NodeOutput();
        val.node = s.readInt();
        int output_len = s.readInt();
        byte[] output_bytes = new byte[output_len];
        for (int output_i = 0; output_i < output_len; output_i++)
        {
            output_bytes[output_i] = s.readByte();
        }
        val.output = new String(output_bytes);
        val.type = OutputType.readFrom(s);
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeInt(this.node);
        s.writeInt(this.output.length());
        s.writeBytes(this.output);
        this.type.writeInto(s);
    }
}
