package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum Controller {

    Bearing, Pitch, Depth, ManualOverride;

    public static Controller readFrom(LEDataInputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 0:
                return Bearing;
            case 1:
                return Pitch;
            case 2:
                return Depth;
            case 3:
                return ManualOverride;
            default:
                throw new IllegalArgumentException("Unrecognized Controller value: " + val);
        }
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case Bearing:
                s.writeByte(0);
                break;
            case Pitch:
                s.writeByte(1);
                break;
            case Depth:
                s.writeByte(2);
                break;
            case ManualOverride:
                s.writeByte(3);
                break;
        }
    }
}


