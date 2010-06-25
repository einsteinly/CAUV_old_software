package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum CameraID {

    Forward, Down, Sonar, File;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case Forward:
                s.writeByte(1);
            case Down:
                s.writeByte(2);
            case Sonar:
                s.writeByte(4);
            case File:
                s.writeByte(5);
        }
    }

    public static CameraID readFrom(DataOutputStream s) throws IOException {
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

}


