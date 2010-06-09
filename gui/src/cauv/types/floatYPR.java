package cauv.types;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

public class floatYPR {
    public float yaw;
    public float pitch;
    public float roll;

    public floatYPR() {
    }

    public static floatYPR readFrom(DataInputStream s) throws IOException {
        floatYPR val = new floatYPR();
        val.yaw = s.readFloat();
        val.pitch = s.readFloat();
        val.roll = s.readFloat();
        return val;
    }

    public void writeInto(DataOutputStream s) throws IOException {
        floatYPR val = this;
        s.writeFloat(val.yaw);
        s.writeFloat(val.pitch);
        s.writeFloat(val.roll);
    }
}
