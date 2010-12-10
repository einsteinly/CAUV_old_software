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
    SonarInput = 25,
    BroadcastImage = 26,
    VideoFileOutput = 27,
    Invert = 28,
    KMeans = 29,
    MixValue = 30,
    Crop = 31,
    GrabCut = 32,
    Histogram = 33,
    HistogramSegmentation = 34,
    Centre = 35,
    QuickSegment = 36,
    ThresholdMask = 37,
    RunningAverage = 38
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
enum Controller : int8 {
    Bearing = 0,
    Pitch = 1,
    Depth = 2,
    ManualOverride = 3
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
    musecs : int32;
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
struct MotorDemand
{
    prop: float;
    hbow: float;
    vbow: float;
    hstern: float;
    vstern : float;
}
struct Line
{
    centre : floatXYZ;
    angle : float;
}
struct Circle
{
    centre : floatXYZ;
    radius : float;
}
struct MotorMap
{
    zeroPlus : int32;
    zeroMinus : int32;
    maxPlus : int32;
    maxMinus : int32;
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
        foreOffset : float;
        foreMultiplier : float;
        aftOffset : float;
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

    message StateRequest : 82
    {
    }

    message Script : 102
    {
        script : string;
        timeout : float;
    }

    message MotorRampRate : 83
    {
        maxDelta : int32;
        updatesPerSecond : int32;
    }

    message SetMotorMap : 84
    {
        motor : MotorID;
        mapping : MotorMap;
    }

	message ResetMCB : 85
	{
    }
}
group state
{
    message State : 81
    {
	    orientation : floatYPR;
    }
}

group telemetry
{
	message Telemetry : 3
	{
	    orientation : floatYPR;
        depth : float;
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

group processing
{
    message HoughLines : 130
    {
         lines : list<Line>;
    }

    message HoughCircles : 131
    {
        circles : list<Circle>;
    }
    
    message Histogram : 132
    {
        bins : list<float>;
        type : string;
    }

    message Centre : 133
    {
        name : string;
        x : float;
        y : float;
    }
}

group gui
{
    message ControllerState : 100
    {
        contoller : Controller;
        mv : float;
        error : float;
        derror : float;
        ierror : float;
        
        demand : MotorDemand;
    }
   
    message MotorState : 101
    {
        demand : MotorDemand;
    }

    message ScriptResponse : 103
    {
        response : string;
    }
}

group pl_gui
{ 
    message NodeAdded : 115
    {
        nodeId : int32;
        nodeType : NodeType;
        inputs : map<string, NodeOutput>;
        outputs : map<string, list<NodeInput> >;
        params : map<string, NodeParamValue>;
    }

    message NodeRemoved : 116
    {
        nodeId : int32;
    }

    message NodeParameters : 117
    {
        nodeId : int32;
        params : map<string, NodeParamValue>;
    }

    message GraphDescription : 118
    {
        nodeTypes : map<int32, NodeType>;
        nodeInputs : map<int32, map<string, NodeOutput> >;
        nodeOutputs : map<int32, map<string, list<NodeInput> > >;
        nodeParams : map<int32, map<string, NodeParamValue> >;
    }

    message ArcAdded : 119
    {
        from : NodeOutput;
        to : NodeInput;
    }

    message ArcRemoved : 120
    {
        from : NodeOutput;
        to : NodeInput;
    }

    message Status : 121
    {
        nodeId : int32;
        status : NodeStatus;
    }

    message InputStatus : 122
    {
        nodeId : int32;
        inputId : string;
        status : NodeIOStatus;
    }
    
    message OutputStatus : 123
    {
        nodeId : int32;
        outputId : string;
        status : NodeIOStatus;
    }
    
	message GuiImage : 124
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

group ai
{
    message AI : 200
    {
        msg : string;
    }
}


