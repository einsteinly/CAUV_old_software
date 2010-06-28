enum DebugType : int8 {
    Trace = 0,
    Debug = 1,
    Error = 2
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
    ConvertColour = 10,
    GuiOutput = 11,
    HoughCircles = 12,
    GaussianBlur = 13,
    MedianFilter = 14,
    BilateralFilter = 15,
    SplitRGB = 16,
    CombineRGB = 17,
    SplitHSV = 18,
    SplitYUV = 19,
    CombineYUV = 20,
    CombineHSV = 21,
    Levels = 22,
    Mix = 23,
    Percentile = 24,
    SonarInput = 25
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
struct NodeParamValue
{
    type : ParamType;
    intValue : int32;
    floatValue : float;
    stringValue : string;
}
struct SonarDataLine
{
    data: list<byte>;
    bearing: int32;
    bearingRange: int32;
    range: int32;
}



group debug
{
	message Debug : 0
	{
        type : DebugType;
	    msg : string;
    }
    message DebugLevel : 1
    {
        level : int32;
    }
}

group control
{
	message Motor : 2
	{
	    motorId : MotorID;
	    speed : int8;
    }

    message BearingAutopilotEnabled : 60
    {
        enabled : bool;
        target : float;
    }
    message BearingAutopilotParams : 70
    {
        Kp : float;
        Ki : float;
        Kd : float;
        scale : float;
    }

    message DepthAutopilotEnabled : 61
    {
        enabled : bool;
        target : float;
    }
    message DepthAutopilotParams : 71
    {
        Kp : float;
        Ki : float;
        Kd : float;
        scale : float;
    }

    message DepthCalibration : 80
    {
        foreMultiplier : float;
        aftMultiplier : float;
    }

    message PitchAutopilotEnabled : 62
    {
        enabled : bool;
        target : float;
    }
    message PitchAutopilotParams : 72
    {
        Kp : float;
        Ki : float;
        Kd : float;
        scale : float;
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
group sonarout
{
	message SonarData : 30
	{
        line : SonarDataLine;
    }
}
group sonarctl
{
	message SonarControl : 32
	{
        direction : uint16;
        width : uint16;
        gain : uint8;
        range : uint32;
        radialRes : uint32;
        angularRes : uint8;
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


group mcb
{
	message Alive : 40
	{
    }
}

group pressure
{
	message Pressure : 50
	{
	    fore : uint16;
	    aft : uint16;
    }
}

