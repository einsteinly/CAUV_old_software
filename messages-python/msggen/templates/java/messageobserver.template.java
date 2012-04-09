package $package;

public class MessageObserver
{
    protected MessageObserver() {}
    
    #for $g in $groups
    #for $m in $g.messages
    #set $className = $m.name + "Message"
    public void on${className}($className m) {}
    #end for
    #end for
}
