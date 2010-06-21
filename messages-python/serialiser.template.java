package ${package};

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

class Serialiser
{
    #for $t in $mapToBaseType($base_types)
    #if $t.name == "string"
    public static $toJavaType($t) read${readwritesuffix($t)}From(DataInputStream s)
    {
        int len = s.readInt();
        byte[] bytes = new byte[len];
        for (int i = 0; i < len; i++)
            bytes[i] = s.readByte();
        return new String(bytes);
    }
    public static void write${readwritesuffix($t)}Into(DataOutputStream s, $toJavaType($t) val)
    {
        s.writeInt(val.length());
        s.writeBytes(val);
    }
    #else
    public static $toJavaType($t) read${readwritesuffix($t)}From(DataInputStream s)
    {
        return s.${dataFuncs[$t.name].read}();
    }
    public static void write${readwritesuffix($t)}Into(DataOutputStream s, $toJavaType($t) val)
    {
        s.${dataFuncs[$t.name].write}(val);
    }
    #end if 
    #end for

    #for $s in $structs
    public static $toJavaType($s) read${readwritesuffix($s)}From(DataInputStream s)
    {
        #for $f in $s.fields
        $f.name = read${readwritesuffix($f.type)}From(s);
        #end for
    }
    public static void write${readwritesuffix($s)}Into(DataOutputStream s, $toJavaType($s) val)
    {
        #for $f in $s.fields
        write${readwritesuffix($f.type)}Into(s, val.$f.name);
        #end for
    }
    #end for
    
    #for $e in $enums
    public static $toJavaType($e) read${readwritesuffix($e)}From(DataInputStream s)
    {
        switch (
        #for $f in $s.fields
        $f.name = read${readwritesuffix($f.type)}From(s);
        #end for
    }
    public static void write${readwritesuffix($s)}Into(DataOutputStream s, $toJavaType($s) val)
    {
        #for $f in $s.fields
        write${readwritesuffix($f.type)}Into(s, val.$f.name);
        #end for
    }
    #end for
}
