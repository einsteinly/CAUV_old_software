package cauv.messaging;

import java.util.LinkedList;
import java.util.Map;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;
import cauv.utils.*;

public class PitchAutopilotParamsMessage extends Message {
    int m_id = 72;
    public float Kp;
    public float Ki;
    public float Kd;
    public float scale;

    public void Kp(float Kp){
        this.Kp = Kp;
    }
    public float Kp(){
        return this.Kp;
    }

    public void Ki(float Ki){
        this.Ki = Ki;
    }
    public float Ki(){
        return this.Ki;
    }

    public void Kd(float Kd){
        this.Kd = Kd;
    }
    public float Kd(){
        return this.Kd;
    }

    public void scale(float scale){
        this.scale = scale;
    }
    public float scale(){
        return this.scale;
    }


    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        LEDataOutputStream s = new LEDataOutputStream(bs);
        s.writeInt(m_id);

        s.writeFloat(this.Kp);
        s.writeFloat(this.Ki);
        s.writeFloat(this.Kd);
        s.writeFloat(this.scale);

        return bs.toByteArray();
    }

    public PitchAutopilotParamsMessage(){
        super(72, "control");
    }

    public PitchAutopilotParamsMessage(Float Kp, Float Ki, Float Kd, Float scale) {
        super(72, "control");
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.scale = scale;
    }

    public PitchAutopilotParamsMessage(byte[] bytes) throws IOException {
        super(72, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        LEDataInputStream s = new LEDataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create PitchAutopilotParamsMessage with invalid id");
        }

        this.Kp = s.readFloat();
        this.Ki = s.readFloat();
        this.Kd = s.readFloat();
        this.scale = s.readFloat();
    }
}
