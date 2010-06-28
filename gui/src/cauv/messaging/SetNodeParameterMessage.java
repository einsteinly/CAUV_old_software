package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class SetNodeParameterMessage extends Message {
    int m_id = 8;
    public int nodeId;
    public String paramId;
    public NodeParamValue value;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }

    public void paramId(String paramId){
        this.paramId = paramId;
    }
    public String paramId(){
        return this.paramId;
    }

    public void value(NodeParamValue value){
        this.value = value;
    }
    public NodeParamValue value(){
        return this.value;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        s.writeInt(this.paramId.length());
        s.writeBytes(this.paramId);
        this.value.writeInto(s);

        return bs.toByteArray();
    }

    public SetNodeParameterMessage(){
        super(8, "pipeline");
    }

    public SetNodeParameterMessage(Integer nodeId, String paramId, NodeParamValue value) {
        super(8, "pipeline");
        this.nodeId = nodeId;
        this.paramId = paramId;
        this.value = value;
    }

    public SetNodeParameterMessage(byte[] bytes) throws IOException {
        super(8, "pipeline");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create SetNodeParameterMessage with invalid id");
        }

        this.nodeId = s.readInt();
        int paramId_len = s.readInt();
        byte[] paramId_bytes = new byte[paramId_len];
        for (int paramId_i = 0; paramId_i < paramId_len; paramId_i++)
        {
            paramId_bytes[paramId_i] = s.readByte();
        }
        this.paramId = new String(paramId_bytes);
        this.value = NodeParamValue.readFrom(s);
    }
}
