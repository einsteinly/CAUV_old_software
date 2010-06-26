package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class NodeInput {
    public int32 node;
    public string input;

    public NodeInput()
    {
    }

    public static NodeInput readFrom(DataOutputStream s) throws IOException {
        NodeInput val = new NodeInput();
        s.writeInt(val.node);
        s.writeInt(val.input.length());
        s.writeBytes(val.input);
        return val;
    }

    public void writeInto(DataInputStream s) throws IOException {
        this.node = s.readInt();
        int input_len = s.readInt();
        byte[] input_bytes = new byte[input_len];
        for (int input_i = 0; input_i < input_len; input_i++)
        {
            input_bytes[input_i] = s.readByte();
        }
        this.input = new String(input_bytes);
    }
}
