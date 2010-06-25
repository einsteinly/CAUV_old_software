package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class NodeAddedMessage extends Message {
    int m_id = 15;
    public int nodeId;
    public NodeType nodeType;
    public HashMap< String, NodeOutput > inputs;
    public HashMap< String, Vector< NodeInput > > outputs;
    public HashMap< String, NodeParamValue > params;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }

    public void nodeType(NodeType nodeType){
        this.nodeType = nodeType;
    }
    public NodeType nodeType(){
        return this.nodeType;
    }

    public void inputs(HashMap< String, NodeOutput > inputs){
        this.inputs = inputs;
    }
    public HashMap< String, NodeOutput > inputs(){
        return this.inputs;
    }

    public void outputs(HashMap< String, Vector< NodeInput > > outputs){
        this.outputs = outputs;
    }
    public HashMap< String, Vector< NodeInput > > outputs(){
        return this.outputs;
    }

    public void params(HashMap< String, NodeParamValue > params){
        this.params = params;
    }
    public HashMap< String, NodeParamValue > params(){
        return this.params;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        this.nodeType.writeInto(s);
        s.writeLong(this.inputs.size());
        for (Map.Entry<String, NodeOutput> inputs_i : this.inputs)
        {
            String inputs_key = this.inputs.getKey();
            s.writeInt(inputs_key.length());
            s.writeBytes(inputs_key);
            NodeOutput inputs_val = this.inputs.getValue();
            inputs_val.writeInto(s);
        }
        s.writeLong(this.outputs.size());
        for (Map.Entry<String, Vector< NodeInput >> outputs_i : this.outputs)
        {
            String outputs_key = this.outputs.getKey();
            s.writeInt(outputs_key.length());
            s.writeBytes(outputs_key);
            Vector< NodeInput > outputs_val = this.outputs.getValue();
            s.writeLong(outputs_val.size());
            for (int outputs_val_i = 0; outputs_val_i < outputs_val.size(); outputs_val_i++)
            {
                NodeInput outputs_val_val = outputs_val.get(outputs_val_i);
                outputs_val_val.writeInto(s);
            }
        }
        s.writeLong(this.params.size());
        for (Map.Entry<String, NodeParamValue> params_i : this.params)
        {
            String params_key = this.params.getKey();
            s.writeInt(params_key.length());
            s.writeBytes(params_key);
            NodeParamValue params_val = this.params.getValue();
            params_val.writeInto(s);
        }

        return bs.toByteArray();
    }

    public NodeAddedMessage(){
        super(15, "pl_gui");
    }

    public NodeAddedMessage(int nodeId, NodeType nodeType, HashMap< String, NodeOutput > inputs, HashMap< String, Vector< NodeInput > > outputs, HashMap< String, NodeParamValue > params) {

        this.nodeId = nodeId;
        this.nodeType = nodeType;
        this.inputs = inputs;
        this.outputs = outputs;
        this.params = params;
    }

    public NodeAddedMessage(byte[] bytes) throws IOException {
        super(15, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create NodeAddedMessage with invalid id");
        }

        this.nodeId = s.readInt();
        this.nodeType.readFrom(s);
        this.inputs = new HashMap< String, NodeOutput >();
        long inputs_len = s.readLong();
        for (int inputs_i = 0; inputs_i < inputs_len; inputs_i++)
        {
            String inputs_key;
            int inputs_key_len = s.readInt();
            byte[] inputs_key_bytes = new byte[inputs_key_len];
            for (int inputs_key_i = 0; inputs_key_i < inputs_key_len; inputs_key_i++)
            {
                inputs_key_bytes[inputs_key_i] = s.readByte();
            }
            inputs_key = new String(inputs_key_bytes);
            NodeOutput inputs_val;
            inputs_val.readFrom(s);
            this.inputs.put(inputs_key, inputs_val);
        }
        this.outputs = new HashMap< String, Vector< NodeInput > >();
        long outputs_len = s.readLong();
        for (int outputs_i = 0; outputs_i < outputs_len; outputs_i++)
        {
            String outputs_key;
            int outputs_key_len = s.readInt();
            byte[] outputs_key_bytes = new byte[outputs_key_len];
            for (int outputs_key_i = 0; outputs_key_i < outputs_key_len; outputs_key_i++)
            {
                outputs_key_bytes[outputs_key_i] = s.readByte();
            }
            outputs_key = new String(outputs_key_bytes);
            Vector< NodeInput > outputs_val;
            outputs_val = new Vector< NodeInput >();
            long $(len)s = s.readLong();
            for (int outputs_val_i = 0; outputs_val_i < outputs_val_len; outputs_val_i++)
            {
                NodeInput outputs_val_val;
                outputs_val_val.readFrom(s);
                outputs_val.add(outputs_val_i, outputs_val_val);
            }
            this.outputs.put(outputs_key, outputs_val);
        }
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
            params_val.readFrom(s);
            this.params.put(params_key, params_val);
        }
    }
}
