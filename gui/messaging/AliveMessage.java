package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class AliveMessage extends Message {
    int m_id = 40;


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);


        return bs.toByteArray();
    }

    public AliveMessage(){
        super(40, "mcb");
    }

    public AliveMessage() {

    }

    public AliveMessage(byte[] bytes) throws IOException {
        super(40, "mcb");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create AliveMessage with invalid id");
        }

    }
}
