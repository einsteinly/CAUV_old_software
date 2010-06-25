package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class NodeRemovedMessage extends Message {
    int m_id = 16;
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

    public NodeRemovedMessage(){
        super(16, "pl_gui");
    }

    public NodeRemovedMessage(int nodeId) {
        super(16, "pl_gui");
        this.nodeId = nodeId;
    }

    public NodeRemovedMessage(byte[] bytes) throws IOException {
        super(16, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create NodeRemovedMessage with invalid id");
        }

        this.nodeId = s.readInt();
    }
}
