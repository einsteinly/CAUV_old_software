package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class NodeRemovedMessage extends Message {
    int m_id = 116;
    protected int nodeId;

    private byte[] bytes;

    public void nodeId(int nodeId) {
        deserialise();
        this.nodeId = nodeId;
    }
    public int nodeId() {
        deserialise();
        return this.nodeId;
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

            return bs.toByteArray();
        }
    }

    public NodeRemovedMessage(){
        super(116, "pl_gui");
        this.bytes = null;
    }

    public NodeRemovedMessage(Integer nodeId) {
        super(116, "pl_gui");
        this.bytes = null;

        this.nodeId = nodeId;
    }

    public NodeRemovedMessage(byte[] bytes) {
        super(116, "pl_gui");
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
                    throw new IllegalArgumentException("Attempted to create NodeRemovedMessage with invalid id");
                }

                this.nodeId = s.readInt();

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
