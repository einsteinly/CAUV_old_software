struct floatYPR
{
    yaw : float;
    pitch : float;
    roll : float;
}
struct floatXYZ
{
    x : float;
    y : float;
    z : float;
}
struct NodeInput
{
    node : int32;
    input : string;
}
struct NodeOutput
{
    node : int32;
    output : string;
}
struct NodeInputArc
{
    input : string;
    src : NodeOutput;
}
struct NodeOutputArc
{
    dst : NodeInput;
    output : string;
}
struct TimeStamp
{
    secs : int32;
    msecs : int32;
}
struct NodeParamValue{
    type : int32;
    intValue : int32;
    floatValue : float;
    stringValue : string;
}

enum MotorID : int8 { Prop = 1, HBow = 2, VBow = 4, HStern = 8, VStern = 16 }
enum CameraID: int8 { Forward = 1, Down = 2, Sonar = 4, File = 5 }
enum NodeType : int8 {
    Invalid = 0,
    Copy = 1,
    Resize = 2,
    FileInput = 3,
    FileOutput = 4,
    LocalDisplay = 5,
    CameraInput = 6,
    NetInput = 7,
    HoughLines = 8,
    Canny = 9,
    ConvertColour = 10
}
enum ParamType : int8 {
    Int32 = 0,
    Float = 1,
    String = 2
}

group control
{
	message Motor : 2
	{
	    motorId : MotorID;
	    speed : int8;
    }
}
group telemetry
{
	message Telemetry : 3
	{
	    orientation : floatYPR;
    }
}
group image
{
	message Image : 4
	{
	    source : CameraID;
	    image : Image;
        time : TimeStamp;
    }
}
group pipeline
{
    message AddNode : 5
    {
        nodeType : NodeType;
        parents : list<NodeInputArc>;
        children : list<NodeOutputArc>;
    }

    message RemoveNode : 6
    {
        nodeId : int32;
    }

    message ClearPipeline : 7
    {
    }

    message SetNodeParameter : 8
    {
        nodeId : int32;
        paramId : string;
        value : NodeParamValue;
    }

    message AddArc : 9
    {
        from : NodeOutput;
        to : NodeInput;
    }
}

group pl_gui
{ 
    message NodeAdded : 10
    {
        nodeId : int32;
        nodeType : NodeType;
        inputs : map<string, NodeOutput>;
        outputs : map<string, list<NodeInput> >;
    }

    message NodeRemoved : 11
    {
        nodeId : int32;
    }

    message NodeParameters : 12
    {
        nodeId : int32;
        values : map<string, NodeParamValue>;
    }

    message GraphDescription : 13
    {
        nodeTypes : map<int32, NodeType>;
        nodeInputs : map<int32, map<string, NodeOutput> >;
        nodeOutputs : map<int32, map<string, list<NodeInput> > >;
        nodeParams : map<int32, map<string, NodeParamValue> >;
    }

    message ArcAdded : 14
    {
        from : NodeOutput;
        to : NodeInput;
    }

    message ArcRemoved : 15
    {
        from : NodeOutput;
        to : NodeInput;
    }
}

