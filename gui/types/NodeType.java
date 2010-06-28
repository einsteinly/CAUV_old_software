package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public enum NodeType {

    Invalid, Copy, Resize, FileInput, FileOutput, LocalDisplay, CameraInput, NetInput, HoughLines, Canny, ConvertColour, GuiOutput, HoughCircles, GaussianBlur, MedianFilter, BilateralFilter, SplitRGB, CombineRGB, SplitHSV, SplitYUV, CombineYUV, CombineHSV, Levels, Mix, Percentile, SonarInput;

    public void writeInto(DataOutputStream s) throws IOException {
        switch (this) {
            case Invalid:
                s.writeByte(0);
            case Copy:
                s.writeByte(1);
            case Resize:
                s.writeByte(2);
            case FileInput:
                s.writeByte(3);
            case FileOutput:
                s.writeByte(4);
            case LocalDisplay:
                s.writeByte(5);
            case CameraInput:
                s.writeByte(6);
            case NetInput:
                s.writeByte(7);
            case HoughLines:
                s.writeByte(8);
            case Canny:
                s.writeByte(9);
            case ConvertColour:
                s.writeByte(10);
            case GuiOutput:
                s.writeByte(11);
            case HoughCircles:
                s.writeByte(12);
            case GaussianBlur:
                s.writeByte(13);
            case MedianFilter:
                s.writeByte(14);
            case BilateralFilter:
                s.writeByte(15);
            case SplitRGB:
                s.writeByte(16);
            case CombineRGB:
                s.writeByte(17);
            case SplitHSV:
                s.writeByte(18);
            case SplitYUV:
                s.writeByte(19);
            case CombineYUV:
                s.writeByte(20);
            case CombineHSV:
                s.writeByte(21);
            case Levels:
                s.writeByte(22);
            case Mix:
                s.writeByte(23);
            case Percentile:
                s.writeByte(24);
            case SonarInput:
                s.writeByte(25);
        }
    }

    public static NodeType readFrom(DataOutputStream s) throws IOException {
        byte val = s.readByte();
        switch (val) {
            case 0:
                return Invalid;
            case 1:
                return Copy;
            case 2:
                return Resize;
            case 3:
                return FileInput;
            case 4:
                return FileOutput;
            case 5:
                return LocalDisplay;
            case 6:
                return CameraInput;
            case 7:
                return NetInput;
            case 8:
                return HoughLines;
            case 9:
                return Canny;
            case 10:
                return ConvertColour;
            case 11:
                return GuiOutput;
            case 12:
                return HoughCircles;
            case 13:
                return GaussianBlur;
            case 14:
                return MedianFilter;
            case 15:
                return BilateralFilter;
            case 16:
                return SplitRGB;
            case 17:
                return CombineRGB;
            case 18:
                return SplitHSV;
            case 19:
                return SplitYUV;
            case 20:
                return CombineYUV;
            case 21:
                return CombineHSV;
            case 22:
                return Levels;
            case 23:
                return Mix;
            case 24:
                return Percentile;
            case 25:
                return SonarInput;
            default:
                throw new IllegalArgumentException("Unrecognized NodeType value: " + val);
        }
    }

}


