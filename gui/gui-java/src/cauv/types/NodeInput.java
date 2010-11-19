package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*;

public class NodeInput {
    public int node;
    public String input;

    public NodeInput()
    {
    }

    public static NodeInput readFrom(LEDataInputStream s) throws IOException {
        NodeInput val = new NodeInput();
        val.node = s.readInt();
        int input_len = s.readInt();
        byte[] input_bytes = new byte[input_len];
        for (int input_i = 0; input_i < input_len; input_i++)
        {
            input_bytes[input_i] = s.readByte();
        }
        val.input = new String(input_bytes);
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        s.writeInt(this.node);
        s.writeInt(this.input.length());
        s.writeBytes(this.input);
    }
}
