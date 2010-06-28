package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

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
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        this.status.writeInto(s);

        return bs.toByteArray();
    }

    public StatusMessage(){
        super(21, "pl_gui");
    }

    public StatusMessage(int nodeId, NodeStatus status) {

        this.nodeId = nodeId;
        this.status = status;
    }

    public StatusMessage(byte[] bytes) throws IOException {
        super(21, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create StatusMessage with invalid id");
        }

        this.nodeId = s.readInt();
        this.status.readFrom(s);
    }
}
