package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class StatusMessage extends Message {
    int m_id = 21;
    public int nodeId;
    public NodeStatus status;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }

    public void status(NodeStatus status){
        this.status = status;
    }
    public NodeStatus status(){
        return this.status;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        this.status.writeInto(s);

        return bs.toByteArray();
    }

    public StatusMessage(){
        super(21, "pl_gui");
    }

    public StatusMessage(Integer nodeId, NodeStatus status) {
        super(21, "pl_gui");
        this.nodeId = nodeId;
        this.status = status;
    }

    public StatusMessage(byte[] bytes) throws IOException {
        super(21, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create StatusMessage with invalid id");
        }

        this.nodeId = s.readInt();
        this.status = NodeStatus.readFrom(s);
    }
}
