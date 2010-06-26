package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class InputStatusMessage extends Message {
    int m_id = 22;
    public int nodeId;
    public String inputId;
    public NodeIOStatus status;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }

    public void inputId(String inputId){
        this.inputId = inputId;
    }
    public String inputId(){
        return this.inputId;
    }

    public void status(NodeIOStatus status){
        this.status = status;
    }
    public NodeIOStatus status(){
        return this.status;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        s.writeInt(this.inputId.length());
        s.writeBytes(this.inputId);
        this.status.writeInto(s);

        return bs.toByteArray();
    }

    public InputStatusMessage(){
        super(22, "pl_gui");
    }

    public InputStatusMessage(int nodeId, String inputId, NodeIOStatus status) {
        super(22, "pl_gui");
        this.nodeId = nodeId;
        this.inputId = inputId;
        this.status = status;
    }

    public InputStatusMessage(byte[] bytes) throws IOException {
        super(22, "pl_gui");
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
    }
}
