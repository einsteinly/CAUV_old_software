package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class StatusMessage extends Message {
    int m_id = 121;
    protected int nodeId;
    protected NodeStatus status;

    private byte[] bytes;

    public void nodeId(int nodeId) {
        deserialise();
        this.nodeId = nodeId;
    }
    public int nodeId() {
        deserialise();
        return this.nodeId;
    }

    public void status(NodeStatus status) {
        deserialise();
        this.status = status;
    }
    public NodeStatus status() {
        deserialise();
        return this.status;
    }


    public byte[] toBytes() throws IOException {
        if (bytes != null)
        {
            return bytes;
        }
        else
        {
            ByteArrayOutputStream bs = new ByteArrayOutputStream();
            LEDataOutputStream s = new LEDataOutputStream(bs);
            s.writeInt(m_id);

            s.writeInt(this.nodeId);
            this.status.writeInto(s);

            return bs.toByteArray();
        }
    }

    public StatusMessage(){
        super(121, "pl_gui");
        this.bytes = null;
    }

    public StatusMessage(Integer nodeId, NodeStatus status) {
        super(121, "pl_gui");
        this.bytes = null;

        this.nodeId = nodeId;
        this.status = status;
    }

    public StatusMessage(byte[] bytes) {
        super(121, "pl_gui");
        this.bytes = bytes;
    }

    public void deserialise() {
        try { 
            if (bytes != null)
            {
                ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
                LEDataInputStream s = new LEDataInputStream(bs);
                int buf_id = s.readInt();
                if (buf_id != m_id)
                {
                    throw new IllegalArgumentException("Attempted to create StatusMessage with invalid id");
                }

                this.nodeId = s.readInt();
                this.status = NodeStatus.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
