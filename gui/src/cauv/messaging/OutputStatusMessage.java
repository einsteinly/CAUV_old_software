package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class OutputStatusMessage extends Message {
    int m_id = 23;
    public int nodeId;
    public String outputId;
    public NodeIOStatus status;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }

    public void outputId(String outputId){
        this.outputId = outputId;
    }
    public String outputId(){
        return this.outputId;
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
        s.writeInt(this.outputId.length());
        s.writeBytes(this.outputId);
        this.status.writeInto(s);

        return bs.toByteArray();
    }

    public OutputStatusMessage(){
        super(23, "pl_gui");
    }

    public OutputStatusMessage(int nodeId, String outputId, NodeIOStatus status) {
        super(23, "pl_gui");
        this.nodeId = nodeId;
        this.outputId = outputId;
        this.status = status;
    }

    public OutputStatusMessage(byte[] bytes) throws IOException {
        super(23, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create OutputStatusMessage with invalid id");
        }

        this.nodeId = s.readInt();
        int outputId_len = s.readInt();
        byte[] outputId_bytes = new byte[outputId_len];
        for (int outputId_i = 0; outputId_i < outputId_len; outputId_i++)
        {
            outputId_bytes[outputId_i] = s.readByte();
        }
        this.outputId = new String(outputId_bytes);
        this.status = NodeIOStatus.readFrom(s);
    }
}
