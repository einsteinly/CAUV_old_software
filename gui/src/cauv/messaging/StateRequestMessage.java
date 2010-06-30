package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class StateRequestMessage extends Message {
    int m_id = 82;


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);


        return bs.toByteArray();
    }

    public StateRequestMessage(){
        super(82, "control");
    }


    public StateRequestMessage(byte[] bytes) throws IOException {
        super(82, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create StateRequestMessage with invalid id");
        }

    }
}
