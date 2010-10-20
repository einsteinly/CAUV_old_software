package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class InputStatusMessage extends Message {
    int m_id = 122;
    protected int nodeId;
    protected String inputId;
    protected NodeIOStatus status;

    private byte[] bytes;

    public void nodeId(int nodeId) {
        deserialise();
        this.nodeId = nodeId;
    }
    public int nodeId() {
        deserialise();
        return this.nodeId;
    }

    public void inputId(String inputId) {
        deserialise();
        this.inputId = inputId;
    }
    public String inputId() {
        deserialise();
        return this.inputId;
    }

    public void status(NodeIOStatus status) {
        deserialise();
        this.status = status;
    }
    public NodeIOStatus status() {
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
            s.writeInt(this.inputId.length());
            s.writeBytes(this.inputId);
            this.status.writeInto(s);

            return bs.toByteArray();
        }
    }

    public InputStatusMessage(){
        super(122, "pl_gui");
        this.bytes = null;
    }

    public InputStatusMessage(Integer nodeId, String inputId, NodeIOStatus status) {
        super(122, "pl_gui");
        this.bytes = null;

        this.nodeId = nodeId;
        this.inputId = inputId;
        this.status = status;
    }

    public InputStatusMessage(byte[] bytes) {
        super(122, "pl_gui");
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
                    throw new IllegalArgumentException("Attempted to create InputStatusMessage with invalid id");
                }

                this.nodeId = s.readInt();
                int inputId_len = s.readInt();
                byte[] inputId_bytes = new byte[inputId_len];
                for (int inputId_i = 0; inputId_i < inputId_len; inputId_i++)
                {
                    inputId_bytes[inputId_i] = s.readByte();
                }
                this.inputId = new String(inputId_bytes);
                this.status = NodeIOStatus.readFrom(s);

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}
