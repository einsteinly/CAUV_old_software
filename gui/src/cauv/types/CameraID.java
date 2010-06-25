package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum CameraID {

    Forward, Down, Sonar, File;

    public static CameraID readFrom(LEDataInputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 1:
                return Forward;
            case 2:
                return Down;
            case 4:
                return Sonar;
            case 5:
                return File;
            default:
                throw new IllegalArgumentException("Unrecognized CameraID value: " + val);
        }
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case Forward:
                s.writeByte(1);
                break;
            case Down:
                s.writeByte(2);
                break;
            case Sonar:
                s.writeByte(4);
                break;
            case File:
                s.writeByte(5);
                break;
        }
    }
}


