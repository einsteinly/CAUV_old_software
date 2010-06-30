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
    public float foreOffset;
    public float foreMultiplier;
    public float aftOffset;
    public float aftMultiplier;

    public void foreOffset(float foreOffset){
        this.foreOffset = foreOffset;
    }
    public float foreOffset(){
        return this.foreOffset;
    }

    public void foreMultiplier(float foreMultiplier){
        this.foreMultiplier = foreMultiplier;
    }
    public float foreMultiplier(){
        return this.foreMultiplier;
    }

    public void aftOffset(float aftOffset){
        this.aftOffset = aftOffset;
    }
    public float aftOffset(){
        return this.aftOffset;
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

        s.writeFloat(this.foreOffset);
        s.writeFloat(this.foreMultiplier);
        s.writeFloat(this.aftOffset);
        s.writeFloat(this.aftMultiplier);

        return bs.toByteArray();
    }

    public DepthCalibrationMessage(){
        super(80, "control");
    }

    public DepthCalibrationMessage(Float foreOffset, Float foreMultiplier, Float aftOffset, Float aftMultiplier) {
        super(80, "control");
        this.foreOffset = foreOffset;
        this.foreMultiplier = foreMultiplier;
        this.aftOffset = aftOffset;
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

        this.foreOffset = s.readFloat();
        this.foreMultiplier = s.readFloat();
        this.aftOffset = s.readFloat();
        this.aftMultiplier = s.readFloat();
    }
}
