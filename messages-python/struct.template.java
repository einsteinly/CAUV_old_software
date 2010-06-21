package $package;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class $name {
    #for $f in $fields
    public $f.type $f.name;
    #end for

    public ${name}()
    {
    }

    public static floatXYZ readFrom(DataInputStream s) throws IOException {
        $name val = new ${name}();
#for $f in $fields
$serialiseJavaType(f.type, f.name, 2, "val.")
#end for
        return val;
    }

    public void writeInto(DataOutputStream s) throws IOException {
#for $f in $fields
$deserialiseJavaType(f.type, f.name, 2, "this.")
#end for
    }
}
