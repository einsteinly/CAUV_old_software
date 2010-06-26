package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

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
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeInt(this.nodeId);
        this.image.writeInto(s);

        return bs.toByteArray();
    }

    public GuiImageMessage(){
        super(24, "pl_gui");
    }

    public GuiImageMessage(int nodeId, Image image) {
        super(24, "pl_gui");
        this.nodeId = nodeId;
        this.image = image;
    }

    public GuiImageMessage(byte[] bytes) throws IOException {
        super(24, "pl_gui");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create GuiImageMessage with invalid id");
        }

        this.nodeId = s.readInt();
        this.image = Image.readFrom(s);
    }
}
