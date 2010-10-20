package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum MotorID {

    Prop, HBow, VBow, HStern, VStern;

    public static MotorID readFrom(LEDataInputStream s) throws IOException {
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

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case Prop:
                s.writeByte(1);
                break;
            case HBow:
                s.writeByte(2);
                break;
            case VBow:
                s.writeByte(4);
                break;
            case HStern:
                s.writeByte(8);
                break;
            case VStern:
                s.writeByte(16);
                break;
        }
    }
}


