\#  This is a generated file, do not edit

import ssrc.spread


\# Type Definitions:
#for $t in $unknown_types
class $t:
    pass
#end for

#for $s in $structs
class $s.name:
    #for $f in $s.fields
    $f.name = $pyTypeInit($f.type)
    #end for

#end for

class Enum:
    def __init__(self, name="", value=0):
        if name != "":
            if name in self.enum_permitted_names:
                self.enum_value = self.enum_permitted_names[name]
            else:
                raise Exception("Invalid enum name: %s" % name)
        elif value in self.enum_permitted_values:
            self.enum_value = value
        else:
            raise Exception("Invalid enum value: %s" % value)
    def __str__(self):
        return self.enum_permitted_values[self.enum_value]
    def __repr__(self):
        return str(self)

#for $e in $enums
class ${e.name}(Enum):
    \# possible values: (TODO: bidirectional map...)
    enum_permitted_names = {
        #for $i, $v in $enumerate($e.values)
            #if $i != len($e.values)-1:
        "$v.name" : $v.value,
            #else
        "$v.name" : $v.value
            #end if
        #end for
    }
    enum_permitted_values = {
        #for $i, $v in $enumerate($e.values)
            #if $i != len($e.values)-1:
        $v.value : "$v.name",
            #else
        $v.value : "$v.name"
            #end if
        #end for
    }
\# initialize instances of possible values:
#for $v in $e.values
${e.name}.${v.name} = ${e.name}("$v.name")
#end for


#end for

\# Message Classes

\# Base message class:

