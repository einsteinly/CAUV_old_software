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
    ConvertColour = 10,
    GuiOutput = 11,
    HoughCircles = 12,
    Blur = 13,
    BilateralFilter = 14,
    SplitRGB = 15,
    CombineRGB = 16,
    SplitHSV = 17,
    SplitYUV = 18,
    CombineYUV = 19,
    CombineHSV = 20,
    Levels = 21,
    Mix = 22,
    Percentile = 23
}
enum NodeStatus : int8 {
    AllowQueue = 1,
    ExecQueued = 2,
    Executing = 4
}
enum NodeIOStatus : int8 {
    New = 1,
    Valid = 2,
    Demanded = 4
}
enum ParamType : int8 {
    Int32 = 0,
    Float = 1,
    String = 2,
    Bool = 3
}
enum OutputType : int8 {
    Image = 0,
    Parameter = 1
}

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
    type : OutputType;
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
    type : ParamType;
    intValue : int32;
    floatValue : float;
    stringValue : string;
}


group debug
{
	message Debug : 0
	{
        type : DebugType;
	    message : string;
    }
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

    message GraphRequest : 10
    {
    }
}

group pl_gui
{ 
    message NodeAdded : 15
    {
        nodeId : int32;
        nodeType : NodeType;
        inputs : map<string, NodeOutput>;
        outputs : map<string, list<NodeInput> >;
        params : map<string, NodeParamValue>;
    }

    message NodeRemoved : 16
    {
        nodeId : int32;
    }

    message NodeParameters : 17
    {
        nodeId : int32;
        params : map<string, NodeParamValue>;
    }

    message GraphDescription : 18
    {
        nodeTypes : map<int32, NodeType>;
        nodeInputs : map<int32, map<string, NodeOutput> >;
        nodeOutputs : map<int32, map<string, list<NodeInput> > >;
        nodeParams : map<int32, map<string, NodeParamValue> >;
    }

    message ArcAdded : 19
    {
        from : NodeOutput;
        to : NodeInput;
    }

    message ArcRemoved : 20
    {
        from : NodeOutput;
        to : NodeInput;
    }

    message Status : 21
    {
        nodeId : int32;
        status : NodeStatus;
    }

    message InputStatus : 22
    {
        nodeId : int32;
        inputId : string;
        status : NodeIOStatus;
    }
    
    message OutputStatus : 23
    {
        nodeId : int32;
        outputId : string;
        status : NodeIOStatus;
    }
    
	message GuiImage : 24
	{
        nodeId : int32;
	    image : Image;
    }
}

