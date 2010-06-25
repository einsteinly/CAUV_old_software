package cauv.messaging;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.types.*;

public class BearingAutopilotParamsMessage extends Message {
    int m_id = 70;
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
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        s.writeFloat(this.Kp);
        s.writeFloat(this.Ki);
        s.writeFloat(this.Kd);
        s.writeFloat(this.scale);

        return bs.toByteArray();
    }

    public BearingAutopilotParamsMessage(){
        super(70, "control");
    }

    public BearingAutopilotParamsMessage(float Kp, float Ki, float Kd, float scale) {

        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.scale = scale;
    }

    public BearingAutopilotParamsMessage(byte[] bytes) throws IOException {
        super(70, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id)
        {
            throw new IllegalArgumentException("Attempted to create BearingAutopilotParamsMessage with invalid id");
        }

        this.Kp = s.readFloat();
        this.Ki = s.readFloat();
        this.Kd = s.readFloat();
        this.scale = s.readFloat();
    }
}
