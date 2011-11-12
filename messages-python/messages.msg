enum DebugType : int8 {
    Info = 0,
    Debug = 1,
    Warning = 2,
    Error = 3
}
enum ImageEncodingType: int8 { RAW_uint8_1 }
enum MotorID : int8 { Prop = 1, HBow = 2, VBow = 4, HStern = 8, VStern = 16 }
enum CameraID: int8 { Forward = 1, Down = 2, Up = 3, Sonar = 4, File = 5, GemSonar = 6 }
enum SonarID: int8 { Seasprite = 1, Gemini = 2 }
enum LightID: int8 { Forward = 1, Down = 2, TO_FIX_NUMVALUES = 5 }
enum NodeType : int8 {
    Invalid,
    Copy,
    Resize,
    FileInput,
    FileOutput,
    LocalDisplay,
    CameraInput,
    NetInput,
    HoughLines,
    Canny,
    ConvertColour,
    GuiOutput,
    HoughCircles,
    GaussianBlur,
    MedianFilter,
    BilateralFilter,
    SplitRGB,
    CombineRGB,
    SplitHSV,
    SplitYUV,
    CombineYUV,
    CombineHSV,
    Levels,
    Mix,
    Percentile,
    SonarInput,
    BroadcastImage,
    VideoFileOutput,
    Invert,
    KMeans,
    MixValue,
    Crop,
    GrabCut,
    Histogram,
    HistogramSegmentation,
    Centre,
    QuickSegment,
    ThresholdMask,
    CornerHarris,
    HistogramSegmentationArb,
    RunningAverage,
    CopyMask,
    FastMedian,
    FASTCorners,
    DrawCorners,
    BroadcastCorners,
    ShiTomasiCorners,
    Pyramid,
    DrawLines,
    BroadcastLines,
    NullParam,
    DrawHistogram,
    IntInput,
    FloatInput,
    StringInput,
    BoolInput,
    BroadcastHistogram,
    Throttle,
    Nop,
    Delay,
    ClampInt,
    ClampFloat,
    Stitch,
    Recogniser,
    SURFCorners,
    BroadcastKeyPoints,
    DrawKeyPoints,
    MathAddMultInt,
    MathAddMultFloat,
    MergeSimilarLines,
    DirectCameraInput,
    SonarSLAM,
    BearingRangeToXY,
    TransformKeyPoints,
    Correlation1D,
    Rotate,
}
enum NodeStatus : int8 {
    None = 0,
    AllowQueue = 1,
    ExecQueued = 2,
    Executing = 4
}
enum NodeInputStatus : int8 {
    Old = 0, New = 1, Invalid = 2
}
enum NodeIOStatus : int8 {
    None = 0,
    New = 1,
    Valid = 2,
    Demanded = 4
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
struct floatXY
{
    x : float;
    y : float;
}
struct NodeInput
{
    node : int32;
    input : string;
    subType : int32;
}
struct LocalNodeInput
{
    input : string;
    subType : int32;
}
struct NodeOutput
{
    node : int32;
    output : string;
    type : OutputType;
    subType : int32;
}
struct LocalNodeOutput
{
    output : string;
    type : OutputType;
    subType : int32;
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
struct SonarDataLine
{
    data: list<byte>;
    bearing: int32;
    bearingRange: int32;
    range: int32;
    scanWidth: int32;
}
struct PolarImage
{
    data: list<byte>;
    // currently only raw uint8_t (isodistant responses in fastest varying index) is supported
    encoding: ImageEncodingType;
    // each bearing in range (-6400 to 6400) * 0x10000, these are the edges of
    // the bins that each iso-bearing beam of data falls into, so there are
    // num_beams + 1 values. The first bin is bearing_bins[0] --
    // bearing_bins[1], and so on.
    // Note that these are *bearings* not angles (ie, clockwise from forwards, not
    // anticlockwise from x axis (wherever that is...))
    bearing_bins: list<int32>;
    // number of bins = round(rangeEnd-rangeStart) / rangeConversion)
    rangeStart: float;
    rangeEnd: float;
    // rangeConversion = length of 1 bin in metres
    rangeConversion: float;
    // time of the ping from which the image is acquired
    timeStamp : TimeStamp;
}
// Can't have these inline in the message otherwise it has too many fields:
struct GeminiTemperatures
{
    dieTemp : float;
    vga1aTemp : float;
    vga1bTemp : float;
    vga2aTemp : float;
    vga2bTemp : float;
    TX1Temp : float;
    TX2Temp : float;
    TX3Temp : float;
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
    length : float;
}
struct Circle
{
    centre : floatXYZ;
    radius : float;
}
struct Corner
{
    centre : floatXYZ;
    size : float;
    angle : float;
    response : float;
}
struct MotorMap
{
    zeroPlus : int32;
    zeroMinus : int32;
    maxPlus : int32;
    maxMinus : int32;
}
struct KeyPoint
{
    pt : floatXY;
    size : float;
    angle : float;
    response : float;
    octave : float;
    class_id : float;
}
variant NodeParamValue
{
    int32, float, string, bool,
    list<Corner>, list<Line>, list<Circle>, list<float>,
    list<KeyPoint>
}
struct ScriptResponse
{
    response : string;
    level : DebugType;
    id : string;
    seq : uint32;
}
struct ScriptExecRequest
{
    script : string;
    timeout : float;
    id : string;
    seq : uint32;
}
class Image : <common/image.h>

group membership
{
    message MembershipChanged : 500
    {
        affectedGroup : string;
    }
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
        Ap : float;
        Ai : float;
        Ad : float;
        thr : float;
        maxError : float;
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
        Ap : float;
        Ai : float;
        Ad : float;
        thr : float;
        maxError : float;
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
        Ap : float;
        Ai : float;
        Ad : float;
        thr : float;
        maxError : float;
    }

    message StateRequest : 82
    {
    }

    message Script : 102
    {
        request : ScriptExecRequest;
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

    message CalibrateNoRotation : 90
    {
        duration : uint16;
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
    message BatteryUse : 86
    {
        estimate_current : float;
        estimate_total : float;
        fraction_remaining : float;
    }
    message ProcessStatus : 87
    {
        process : string;
        status : string;
        cpu : float;
        mem : float;
        threads : uint32;
    }
    message Location : 88
    {
        latitude : double;
        longitude : double;
        altitude : float;
        speed : floatXYZ;
    }
    message GPSLocation : 89
    {
        latitude : double;
        longitude : double;
        altitude : float;
        track : float;
        speed: float;
        climb: float;
    }
    message SonarLocation : 91
    {
        position: floatXY;
    }
}
group image
{
    message Image : 4
    {
        source : CameraID;
        lazy image : Image;
        time : TimeStamp;
    }
}
group sonarout
{
    message SonarData : 30
    {
        line : SonarDataLine;
    }

    message SonarImage : 31
    {
        source : SonarID;
        image : PolarImage;
    }

    message SpeedOfSound : 332
    {
        speedOfSound : float; // m/s
    }

    message GeminiStatus : 333
    {
        sonarId : uint16;
        vccInt : uint16;
        vccAux : uint16;
        dcVolt : uint16;
        // temperatures in degrees C
        currentTemps : GeminiTemperatures;
        // maximum acceptable temperatures:
        criticalTemps : GeminiTemperatures;
        // Other stuff:
        transducerFrequency : float; // in kHz
        linkType : int32; // bit 0 = VDSL, bit 1 = 10Mbps ethernet, bit 2 = 100Mbps, bit 3 = 1000Mbps
        BOOTSTSRegister : uint32; // error codes last two boot attempts
        shutdownStatus : uint16;  // bit 0 = shutdown due to overtemperature
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
        rangeRes : uint32;
        angularRes : uint8;
    }
    message GeminiControl : 33
    {
        range : float;  // in m (0-50)
        gain : float;   // percentage (0-100)
        rangeLines : uint32; // 0-4096: used for range compression setting: the
                             // number of returned lines depends only on range!
        continuous : bool;
        interPingPeriod : float; // seconds (musec resolution)
    }
}
group pipeline
{
    message AddNode : 5
    {
        pipelineName : string;
        nodeType : NodeType;
        parents : list<NodeInputArc>;
        children : list<NodeOutputArc>;
    }

    message RemoveNode : 6
    {
        pipelineName : string;
        nodeId : int32;
    }

    message ClearPipeline : 7
    {
        pipelineName : string;
    }

    message SetNodeParameter : 8
    {
        pipelineName : string;
        nodeId : int32;
        paramId : string;
        value : NodeParamValue;
    }

    message AddArc : 9
    {
        pipelineName : string;
        from : NodeOutput;
        to : NodeInput;
    }

    message RemoveArc : 14
    {
        pipelineName : string;
        from : NodeOutput;
        to : NodeInput;
    }

    message GraphRequest : 10
    {
        pipelineName : string;
    }

    message ForceExecRequest : 11
    {
        pipelineName : string;
        nodeId : int32;
    }

    message PipelineDiscoveryRequest : 12
    {
    }

    message PipelineDiscoveryResponse : 13
    {
        pipelineName : string;
    }
}

group processing
{
    message Lines : 130
    {
        name : string;
        lines : list<Line>;
    }

    message Circles : 131
    {
        name : string;
        circles : list<Circle>;
    }

    message Corners : 132
    {
        name : string;
        corners : list<Corner>;
    }

    message KeyPoints : 135
    {
        name : string;
        keypoints : list<KeyPoint>;
    }

    message Histogram : 133
    {
        name : string;
        bins : list<float>;
    }

    message Centre : 134
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
        kp : float;
        ki : float;
        kd : float;

        demand : MotorDemand;
    }

    message MotorState : 101
    {
        motorId : MotorID;
        speed : int8;
    }

    message ScriptResponse : 103
    {
        response : ScriptResponse;
    }

    message Graphable : 104
    {
        name : string;
        value : float;
    }
}

group pl_gui
{
    message NodeAdded : 115
    {
        pipelineName : string;
        nodeId : int32;
        nodeType : NodeType;
        inputs : map<LocalNodeInput, NodeOutput>;
        outputs : map<LocalNodeOutput, list<NodeInput> >;
        params : map<LocalNodeInput, NodeParamValue>;
    }

    message NodeRemoved : 116
    {
        pipelineName : string;
        nodeId : int32;
    }

    message NodeParameters : 117
    {
        pipelineName : string;
        nodeId : int32;
        params : map<LocalNodeInput, NodeParamValue>;
    }

    message GraphDescription : 118
    {
        pipelineName : string;
        lazy nodeTypes : map<int32, NodeType>;
        lazy nodeInputs : map<int32, map<LocalNodeInput, NodeOutput> >;
        lazy nodeOutputs : map<int32, map<LocalNodeOutput, list<NodeInput> > >;
        lazy nodeParams : map<int32, map<LocalNodeInput, NodeParamValue> >;
    }

    message ArcAdded : 119
    {
        pipelineName : string;
        from : NodeOutput;
        to : NodeInput;
    }

    message ArcRemoved : 120
    {
        pipelineName : string;
        from : NodeOutput;
        to : NodeInput;
    }

    message Status : 121
    {
        pipelineName : string;
        nodeId : int32;
        status : NodeStatus;
    }

    message InputStatus : 122
    {
        pipelineName : string;
        nodeId : int32;
        inputId : string;
        status : NodeIOStatus;
    }

    message OutputStatus : 123
    {
        pipelineName : string;
        nodeId : int32;
        outputId : string;
        status : NodeIOStatus;
    }

    message GuiImage : 124
    {
        pipelineName : string;
        nodeId : int32;
        lazy image : Image;
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
    message AIlog : 201
    {
        msg : string;
    }
}

group external
{
    message Light : 150
    {
        lightId: LightID;
        intensity : uint8;
    }
    message CuttingDevice : 151
    {
        strength : uint8;
    }
    message BatteryStatus : 152
    {
        voltage : uint16;
    }
}
group simulation
{
    message SimPosition : 800
    {
        latitude : double;
        longitude : double;
        altitude : float;
        orientation : floatYPR;
        speed : floatXYZ;
    }
}
