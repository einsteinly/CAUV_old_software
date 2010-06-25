package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class GuiImageMessage extends Message {
    int m_id = 24;
    public int nodeId;
    public Image image;

    public void nodeId(int nodeId){
        this.nodeId = nodeId;
    }
    public int nodeId(){
        return this.nodeId;
    }

    public void image(Image image){
        this.image = image;
    }
    public Image image(){
        return this.image;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        this.image.writeInto(s);

        return bs.toByteArray();
    }

    public GuiImageMessage(){
        super(24, "pl_gui");
    }

    public GuiImageMessage(int nodeId, Image image) {

        this.nodeId = nodeId;
        this.image = image;
    }

    public GuiImageMessage(byte[] bytes) throws IOException {
        super(24, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create GuiImageMessage with invalid id");
        }

        this.nodeId = s.readInt();
        this.image.readFrom(s);
    }
}
