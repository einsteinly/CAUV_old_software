package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class RemoveNodeMessage extends Message {
    int m_id = 6;
    public int nodeId;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);

        return bs.toByteArray();
    }

    public RemoveNodeMessage(){
        super(6, "pipeline");
    }

    public RemoveNodeMessage(int nodeId) {
        super(6, "pipeline");
        this.nodeId = nodeId;
    }

    public RemoveNodeMessage(byte[] bytes) throws IOException {
        super(6, "pipeline");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create RemoveNodeMessage with invalid id");
        }

        this.nodeId = s.readInt();
    }
}
