package $package;

import java.util.LinkedList;
import java.util.Vector;
import java.util.HashMap;
import java.io.*;

public class $name {
    #for $f in $fields
    public $toJavaType($f.type) $f.name;

//serialise
$serialiseJavaType(f.type, f.name, 2, "val.")

//deserialise
$deserialiseJavaType(f.type, f.name, 2, "this.")


    #end for

    public ${name}()
    {
    }
}
