package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class DepthCalibrationMessage extends Message {
    int m_id = 80;
    public float foreMultiplier;
    public float aftMultiplier;

    public void foreMultiplier(float foreMultiplier){
        this.foreMultiplier = foreMultiplier;
    }
    public float foreMultiplier(){
        return this.foreMultiplier;
    }

    public void aftMultiplier(float aftMultiplier){
        this.aftMultiplier = aftMultiplier;
    }
    public float aftMultiplier(){
        return this.aftMultiplier;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeFloat(this.foreMultiplier);
        s.writeFloat(this.aftMultiplier);

        return bs.toByteArray();
    }

    public DepthCalibrationMessage(){
        super(80, "control");
    }

    public DepthCalibrationMessage(Float foreMultiplier, Float aftMultiplier) {
        super(80, "control");
        this.foreMultiplier = foreMultiplier;
        this.aftMultiplier = aftMultiplier;
    }

    public DepthCalibrationMessage(byte[] bytes) throws IOException {
        super(80, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create DepthCalibrationMessage with invalid id");
        }

        this.foreMultiplier = s.readFloat();
        this.aftMultiplier = s.readFloat();
    }
}
