package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum MotorID {

    Prop, HBow, VBow, HStern, VStern;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case Prop:
                s.writeByte(1);
            case HBow:
                s.writeByte(2);
            case VBow:
                s.writeByte(4);
            case HStern:
                s.writeByte(8);
            case VStern:
                s.writeByte(16);
        }
    }

    public static MotorID readFrom(DataOutputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 1:
                return Prop;
            case 2:
                return HBow;
            case 4:
                return VBow;
            case 8:
                return HStern;
            case 16:
                return VStern;
            default:
                throw new IllegalArgumentException("Unrecognized MotorID value: " + val);
        }
    }

}


