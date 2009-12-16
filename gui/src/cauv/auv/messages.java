package cauv.auv;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

class floatYPR {
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

class floatXYZ {
    public float x;
    public float y;
    public float z;

    public floatXYZ() {
    }

    public static floatXYZ readFrom(DataInputStream s) throws IOException {
        floatXYZ val = new floatXYZ();
        val.x = s.readFloat();
        val.y = s.readFloat();
        val.z = s.readFloat();
        return val;
    }

    public void writeInto(DataOutputStream s) throws IOException {
        floatXYZ val = this;
        s.writeFloat(val.x);
        s.writeFloat(val.y);
        s.writeFloat(val.z);
    }
}

enum MotorID {
    PROP, HBOW, VBOW, HSTERN, VSTERN;

    public static MotorID readFrom(DataInputStream s) throws IOException {
        int val = s.readInt();
        switch (val) {
            case 1:
                return PROP;
            case 2:
                return HBOW;
            case 4:
                return VBOW;
            case 8:
                return HSTERN;
            case 16:
                return VSTERN;
            default:
                throw new IllegalArgumentException("Unrecognized MotorID value: " + val);
        }
    }

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case PROP:
                s.writeInt(1);
                break;
            case HBOW:
                s.writeInt(2);
                break;
            case VBOW:
                s.writeInt(4);
                break;
            case HSTERN:
                s.writeInt(8);
                break;
            case VSTERN:
                s.writeInt(16);
                break;
        }
    }

}

enum CameraID {
    FORWARD, DOWN, SONAR;

    public static CameraID readFrom(DataInputStream s) throws IOException {
        int val = s.readInt();
        switch (val) {
            case 1:
                return FORWARD;
            case 2:
                return DOWN;
            case 4:
                return SONAR;
            default:
                throw new IllegalArgumentException("Unrecognized CameraID value: " + val);
        }
    }

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case FORWARD:
                s.writeInt(1);
                break;
            case DOWN:
                s.writeInt(2);
                break;
            case SONAR:
                s.writeInt(4);
                break;
        }
    }

}

abstract class Message {
    public String group() {
        return m_group;
    }

    public int id() {
        return m_id;
    }

    public abstract byte[] toBytes() throws IOException;

    protected int m_id;
    protected String m_group;

    protected Message(int id, String group) {
        m_id = id;
        m_group = group;
    }

    @Override
    public String toString() {
        return this.getClass().getName() + ": {\n\tID: " + this.id() + "\n\tGroup: " + this.group()
                + "\n}";
    }
}

class MotorMessage extends Message {
    protected MotorID m_motorId;
    protected byte m_speed;

    public MotorID motorId() {
        return m_motorId;
    }

    public void motorId(MotorID val) {
        m_motorId = val;
    }

    public byte speed() {
        return m_speed;
    }

    public void speed(byte val) {
        m_speed = val;
    }

    public MotorMessage() {
        super(2, "control");
    }

    public MotorMessage(MotorID motorId, byte speed) {
        super(2, "control");
        m_motorId = motorId;
        m_speed = speed;
    }

    public MotorMessage(byte[] bytes) throws IOException {
        super(2, "control");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id) { throw new IllegalArgumentException(
                "Attempted to create MotorMessage with invalid id"); }

        m_motorId = MotorID.readFrom(s);
        m_speed = s.readByte();
    }

    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        m_motorId.writeInto(s);
        s.writeByte(m_speed);

        return bs.toByteArray();
    }
}

class TelemetryMessage extends Message {
    protected floatYPR m_orientation;

    public floatYPR orientation() {
        return m_orientation;
    }

    public void orientation(floatYPR val) {
        m_orientation = val;
    }

    public TelemetryMessage() {
        super(3, "telemetry");
    }

    public TelemetryMessage(floatYPR orientation) {
        super(3, "telemetry");
        m_orientation = orientation;
    }

    public TelemetryMessage(byte[] bytes) throws IOException {
        super(3, "telemetry");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id) { throw new IllegalArgumentException(
                "Attempted to create TelemetryMessage with invalid id"); }

        m_orientation = floatYPR.readFrom(s);
    }

    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        m_orientation.writeInto(s);

        return bs.toByteArray();
    }
}

class ImageMessage extends Message {
    protected CameraID m_source;
    protected byte m_type;
    protected Vector<Byte> m_image;

    public CameraID source() {
        return m_source;
    }

    public void source(CameraID val) {
        m_source = val;
    }

    public byte type() {
        return m_type;
    }

    public void type(byte val) {
        m_type = val;
    }

    public Vector<Byte> image() {
        return m_image;
    }

    public void image(Vector<Byte> val) {
        m_image = val;
    }

    public ImageMessage() {
        super(4, "image");
    }

    public ImageMessage(CameraID source, byte type, Vector<Byte> image) {
        super(4, "image");
        m_source = source;
        m_type = type;
        m_image = image;
    }

    public ImageMessage(byte[] bytes) throws IOException {
        super(4, "image");
        ByteArrayInputStream bs = new ByteArrayInputStream(bytes);
        DataInputStream s = new DataInputStream(bs);
        int buf_id = s.readInt();
        if (buf_id != m_id) { throw new IllegalArgumentException(
                "Attempted to create ImageMessage with invalid id"); }

        m_source = CameraID.readFrom(s);
        m_type = s.readByte();
        m_image = new Vector<Byte>();
        long image_i2_max = s.readLong();
        for (int i2 = 0; i2 < image_i2_max; i2++) {
            byte i2_val;
            i2_val = s.readByte();
            m_image.add(i2, i2_val);
        }
    }

    public byte[] toBytes() throws IOException {
        ByteArrayOutputStream bs = new ByteArrayOutputStream();
        DataOutputStream s = new DataOutputStream(bs);
        s.writeInt(m_id);

        m_source.writeInto(s);
        s.writeByte(m_type);
        s.writeLong(m_image.size());
        for (int i2 = 0; i2 < m_image.size(); i2++) {
            byte i2_val = m_image.get(i2);
            s.writeByte(i2_val);
        }

        return bs.toByteArray();
    }
}

interface MessageObserver {

    public void onMotorMessage(final MotorMessage m);

    public void onTelemetryMessage(final TelemetryMessage m);

    public void onImageMessage(final ImageMessage m);
};

class MessageSource {
    protected LinkedList<MessageObserver> m_obs = new LinkedList<MessageObserver>();

    protected MessageSource() {
    }

    public void notifyObservers(byte[] b) {
        if (b.length < 4)
            throw new IllegalArgumentException("Buffer too small to contain message id");

        int id = b[3] << 24 | b[2] << 16 | b[1] << 8 | b[0];
        try {
            switch (id) {
                case 2: {
                    MotorMessage m = new MotorMessage(b);
                    for (MessageObserver o : m_obs) {
                        o.onMotorMessage(m);
                    }
                    break;
                }
                case 3: {
                    TelemetryMessage m = new TelemetryMessage(b);
                    for (MessageObserver o : m_obs) {
                        o.onTelemetryMessage(m);
                    }
                    break;
                }
                case 4: {
                    ImageMessage m = new ImageMessage(b);
                    for (MessageObserver o : m_obs) {
                        o.onImageMessage(m);
                    }
                    break;
                }
            }
        } catch (IOException e) {
        }
    }

    public void addObserver(MessageObserver o) {
        m_obs.add(o);
    }

    public void removeObserver(MessageObserver o) {
        m_obs.remove(o);
    }

    public void clearObservers() {
        m_obs.clear();
    }
}
