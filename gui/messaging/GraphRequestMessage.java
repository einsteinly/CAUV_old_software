package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class GraphRequestMessage extends Message {
    int m_id = 10;


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);


        return bs.toByteArray();
    }

    public GraphRequestMessage(){
        super(10, "pipeline");
    }

    public GraphRequestMessage() {

    }

    public GraphRequestMessage(byte[] bytes) throws IOException {
        super(10, "pipeline");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create GraphRequestMessage with invalid id");
        }

    }
}
