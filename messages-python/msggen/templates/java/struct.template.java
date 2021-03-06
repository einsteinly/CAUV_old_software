package $package;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

import ${rootpackage}.utils.*;

public class $s.name {
    #for $f in $s.fields
    public $toJavaType($f.type) $f.name;
    #end for

    public ${s.name}()
    {
    }

    public static ${s.name} readFrom(LEDataInputStream s) throws IOException {
        $s.name val = new ${s.name}();
        #for $f in $s.fields
$deserialiseJavaType(f.type, f.name, 2, "val.")
        #end for
        return val;
    }

    public void writeInto(LEDataOutputStream s) throws IOException {
        #for $f in $s.fields
$serialiseJavaType(f.type, f.name, 2, "this.")
        #end for
    }
}
