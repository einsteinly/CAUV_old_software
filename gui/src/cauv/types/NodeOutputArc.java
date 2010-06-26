package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class NodeOutputArc {
    public NodeInput dst;
    public String output;

    public NodeOutputArc()
    {
    }

    public static NodeOutputArc readFrom(LEDataInputStream s) throws IOException {
        NodeOutputArc val = new NodeOutputArc();
        val.dst = NodeInput.readFrom(s);
        int output_len = s.readInt();
        byte[] output_bytes = new byte[output_len];
        for (int output_i = 0; output_i < output_len; output_i++)
        {
            output_bytes[output_i] = s.readByte();
        }
        val.output = new String(output_bytes);
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        this.dst.writeInto(s);
        s.writeInt(this.output.length());
        s.writeBytes(this.output);
    }
}
