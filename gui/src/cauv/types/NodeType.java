package cauv.types;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import cauv.utils.*; 

public enum NodeType {

    Invalid, Copy, Resize, FileInput, FileOutput, LocalDisplay, CameraInput, NetInput, HoughLines, Canny, ConvertColour, GuiOutput, HoughCircles, GaussianBlur, MedianFilter, BilateralFilter, SplitRGB, CombineRGB, SplitHSV, SplitYUV, CombineYUV, CombineHSV, Levels, Mix, Percentile, SonarInput, BroadcastImage, Invert;

    public static NodeType readFrom(LEDataInputStream s) throws IOException {
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
            case 26:
                return BroadcastImage;
            case 27:
                return Invert;
            default:
                throw new IllegalArgumentException("Unrecognized NodeType value: " + val);
        }
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        switch (this) {
            case Invalid:
                s.writeByte(0);
                break;
            case Copy:
                s.writeByte(1);
                break;
            case Resize:
                s.writeByte(2);
                break;
            case FileInput:
                s.writeByte(3);
                break;
            case FileOutput:
                s.writeByte(4);
                break;
            case LocalDisplay:
                s.writeByte(5);
                break;
            case CameraInput:
                s.writeByte(6);
                break;
            case NetInput:
                s.writeByte(7);
                break;
            case HoughLines:
                s.writeByte(8);
                break;
            case Canny:
                s.writeByte(9);
                break;
            case ConvertColour:
                s.writeByte(10);
                break;
            case GuiOutput:
                s.writeByte(11);
                break;
            case HoughCircles:
                s.writeByte(12);
                break;
            case GaussianBlur:
                s.writeByte(13);
                break;
            case MedianFilter:
                s.writeByte(14);
                break;
            case BilateralFilter:
                s.writeByte(15);
                break;
            case SplitRGB:
                s.writeByte(16);
                break;
            case CombineRGB:
                s.writeByte(17);
                break;
            case SplitHSV:
                s.writeByte(18);
                break;
            case SplitYUV:
                s.writeByte(19);
                break;
            case CombineYUV:
                s.writeByte(20);
                break;
            case CombineHSV:
                s.writeByte(21);
                break;
            case Levels:
                s.writeByte(22);
                break;
            case Mix:
                s.writeByte(23);
                break;
            case Percentile:
                s.writeByte(24);
                break;
            case SonarInput:
                s.writeByte(25);
                break;
            case BroadcastImage:
                s.writeByte(26);
                break;
            case Invert:
                s.writeByte(27);
                break;
        }
    }
}


