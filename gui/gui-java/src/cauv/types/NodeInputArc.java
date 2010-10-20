package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class NodeInputArc {
    public String input;
    public NodeOutput src;

    public NodeInputArc()
    {
    }

    public static NodeInputArc readFrom(LEDataInputStream s) throws IOException {
        NodeInputArc val = new NodeInputArc();
        int input_len = s.readInt();
        byte[] input_bytes = new byte[input_len];
        for (int input_i = 0; input_i < input_len; input_i++)
        {
            input_bytes[input_i] = s.readByte();
        }
        val.input = new String(input_bytes);
        val.src = NodeOutput.readFrom(s);
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeInt(this.input.length());
        s.writeBytes(this.input);
        this.src.writeInto(s);
    }
}
