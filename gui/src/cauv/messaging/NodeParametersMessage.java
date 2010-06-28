package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class NodeParametersMessage extends Message {
    int m_id = 17;
    public int nodeId;
    public HashMap< String, NodeParamValue > params;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }

    public void params(HashMap< String, NodeParamValue > params){
        this.params = params;
    }
    public HashMap< String, NodeParamValue > params(){
        return this.params;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        s.writeLong(this.params.size());
        for (Map.Entry<String, NodeParamValue> params_i : this.params.entrySet())
        {
            String params_key = params_i.getKey();
            s.writeInt(params_key.length());
            s.writeBytes(params_key);
            NodeParamValue params_val = params_i.getValue();
            params_val.writeInto(s);
        }

        return bs.toByteArray();
    }

    public NodeParametersMessage(){
        super(17, "pl_gui");
    }

    public NodeParametersMessage(Integer nodeId, HashMap< String, NodeParamValue > params) {
        super(17, "pl_gui");
        this.nodeId = nodeId;
        this.params = params;
    }

    public NodeParametersMessage(byte[] bytes) throws IOException {
        super(17, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create NodeParametersMessage with invalid id");
        }

        this.nodeId = s.readInt();
        this.params = new HashMap< String, NodeParamValue >();
        long params_len = s.readLong();
        for (int params_i = 0; params_i < params_len; params_i++)
        {
            String params_key;
            int params_key_len = s.readInt();
            byte[] params_key_bytes = new byte[params_key_len];
            for (int params_key_i = 0; params_key_i < params_key_len; params_key_i++)
            {
                params_key_bytes[params_key_i] = s.readByte();
            }
            params_key = new String(params_key_bytes);
            NodeParamValue params_val;
            params_val = NodeParamValue.readFrom(s);
            this.params.put(params_key, params_val);
        }
    }
}
