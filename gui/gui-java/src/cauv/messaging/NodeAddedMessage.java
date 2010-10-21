package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class NodeAddedMessage extends Message {
    int m_id = 115;
    protected int nodeId;
    protected NodeType nodeType;
    protected HashMap< String, NodeOutput > inputs;
    protected HashMap< String, Vector< NodeInput > > outputs;
    protected HashMap< String, NodeParamValue > params;

    private byte[] bytes;

    public void nodeId(int nodeId) {
        deserialise();
        this.nodeId = nodeId;
    }
    public int nodeId() {
        deserialise();
        return this.nodeId;
    }

    public void nodeType(NodeType nodeType) {
        deserialise();
        this.nodeType = nodeType;
    }
    public NodeType nodeType() {
        deserialise();
        return this.nodeType;
    }

    public void inputs(HashMap< String, NodeOutput > inputs) {
        deserialise();
        this.inputs = inputs;
    }
    public HashMap< String, NodeOutput > inputs() {
        deserialise();
        return this.inputs;
    }

    public void outputs(HashMap< String, Vector< NodeInput > > outputs) {
        deserialise();
        this.outputs = outputs;
    }
    public HashMap< String, Vector< NodeInput > > outputs() {
        deserialise();
        return this.outputs;
    }

    public void params(HashMap< String, NodeParamValue > params) {
        deserialise();
        this.params = params;
    }
    public HashMap< String, NodeParamValue > params() {
        deserialise();
        return this.params;
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
            this.nodeType.writeInto(s);
            s.writeInt(this.inputs.size());
            for (Map.Entry<String, NodeOutput> inputs_i : this.inputs.entrySet())
            {
                String inputs_key = inputs_i.getKey();
                s.writeInt(inputs_key.length());
                s.writeBytes(inputs_key);
                NodeOutput inputs_val = inputs_i.getValue();
                inputs_val.writeInto(s);
            }
            s.writeInt(this.outputs.size());
            for (Map.Entry<String, Vector< NodeInput >> outputs_i : this.outputs.entrySet())
            {
                String outputs_key = outputs_i.getKey();
                s.writeInt(outputs_key.length());
                s.writeBytes(outputs_key);
                Vector< NodeInput > outputs_val = outputs_i.getValue();
                s.writeInt(outputs_val.size());
                for (int outputs_val_i = 0; outputs_val_i < outputs_val.size(); outputs_val_i++)
                {
                    NodeInput outputs_val_val = outputs_val.get(outputs_val_i);
                    outputs_val_val.writeInto(s);
                }
            }
            s.writeInt(this.params.size());
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
    }

    public NodeAddedMessage(){
        super(115, "pl_gui");
        this.bytes = null;
    }

    public NodeAddedMessage(Integer nodeId, NodeType nodeType, HashMap< String, NodeOutput > inputs, HashMap< String, Vector< NodeInput > > outputs, HashMap< String, NodeParamValue > params) {
        super(115, "pl_gui");
        this.bytes = null;

        this.nodeId = nodeId;
        this.nodeType = nodeType;
        this.inputs = inputs;
        this.outputs = outputs;
        this.params = params;
    }

    public NodeAddedMessage(byte[] bytes) {
        super(115, "pl_gui");
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
                    throw new IllegalArgumentException("Attempted to create NodeAddedMessage with invalid id");
                }

                this.nodeId = s.readInt();
                this.nodeType = NodeType.readFrom(s);
                this.inputs = new HashMap< String, NodeOutput >();
                long inputs_len = s.readInt();
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
                    inputs_val = NodeOutput.readFrom(s);
                    this.inputs.put(inputs_key, inputs_val);
                }
                this.outputs = new HashMap< String, Vector< NodeInput > >();
                long outputs_len = s.readInt();
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
                    long outputs_val_len = s.readInt();
                    for (int outputs_val_i = 0; outputs_val_i < outputs_val_len; outputs_val_i++)
                    {
                        NodeInput outputs_val_val;
                        outputs_val_val = NodeInput.readFrom(s);
                        outputs_val.add(outputs_val_i, outputs_val_val);
                    }
                    this.outputs.put(outputs_key, outputs_val);
                }
                this.params = new HashMap< String, NodeParamValue >();
                long params_len = s.readInt();
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

                bytes = null;
            }
        }
        catch (IOException e) {}
    }
}