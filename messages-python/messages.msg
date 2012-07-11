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
enum LightID: int8 { Forward = 1, Down = 2, Strobe = 3, }
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
    RenderString,
    MathDivideInt,
    MathDivideFloat,
    GlobalMaximum,
    LocalMaxima,
    SonarShadowFilter,
    PolarImageToXY,
    SonarImageEdge,
    LearnedKeyPoints,
    FirstAboveThreshold,
    BroadcastPoints,
    FitEllipse,
    KMeansPoints,
    DrawEllipses,
    Sobel,
    Blob,
    BearingRangeCrop,
    DrawCircles,
    BroadcastCircles,
    RelabelClusters,
    FitGaussian,
    ReceiveLines,
    MeanStd,
    MathAddMultColour,
    BroadcastEllipses,
    ColourSimilarity,
    BackgroundSubtractor,
    GetNthColour,
    ClampColour,
    SumSquared,
    BroadcastFloat,
}
enum NodeStatus : int8 {
    None = 0,
    AllowQueue = 1,
    ExecQueued = 2,
    Executing = 4,
    WaitingForSync = 8,
    Bad = 16,
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
enum PipelineState : int8 {
    Play,
    Pause,
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
enum InputSchedType : int8 {
    Must_Be_New = 0,
    May_Be_Old = 1,
    Optional = 2
}
enum ScriptCommand : int8 {
    Start,
    Stop,
    Reset,
}
enum AICommand : int8 {
    PauseAll,
    ResumeAll,
    Save,
}
enum SensorUIDBase : int32 {
    None = 0,
    Camera =   0x10,  // cameras are 0x10 +- ID: range [0x1--0x1F]
    Sonar =    0x20,   // sonars are 0x20 + ID: range [0x20--0x3F]
    Network =  0x50, // network is just 0x50, but has a reserved range
    Multiple = 0x70, // multiple sources is just 0x70 (no reserved range)
    File =     0x100,   // files are 0x100 += ?: range[0x100--0xFFF]
}
enum ColourType : int8 {
    RGB,
    BGR,
    ARGB,
    BGRA,
    Grey,
}

enum PCBStatus : int8 {
    OK = 0,
    Alarm = 1,    // fault indicated by unusual data (e.g. spurious pressure reading)
    Error = 2,    // a fault that may affect behaviour, but which should not abort the mission
    Critical = 3, // requires immediate action or damage will be caused (e.g.  short)
    Fatal = 4,    // use for genuinely fatal errors: pressure vessel flooding, all motors dead, etc...
}

enum BoardID : int16 {
    MainControl = 10,
    MotorSupervisor = 11,
    FrontPressureSensor = 12,
    RearPressureSensor = 13,
    ComputerSupervisor = 16,
    //FrontLightsControl = ???
    //MidLightsControl = ???
    //AudioProcessing = ???
}

enum PowerControl : int8 {
    PowerOff = 0,
    PowerOn  = 1,
    Reset    = 2,
}

enum BatteryLocation : int8 {
    Left = 0,
    Right = 1,
}

// used for temperature
enum Component : int8 {
    FrontPressureBoard,
    RearPressureBoard,
    MainControlBoard,
    // !!! TODO: other temperatures (GeminiXXX, GeminiYYY, SecoCPU, SecoGPU, PandaCPU, INS, LBatteryCell1, LBatteryCell2 ...)
    // which come from places other than the can bus
}

enum PowerChannel : int8 {
    PandaBoard,
    SECOBoard,
    ForeCan, // !!! one of these includes the MCB
    AftCan,  // !!! one of these includes the MCB
    Gemini,
    SeaSprite,
    Antenna,
    MotorControl,
    IMU,
    Ethernet,
}

struct floatYPR
{
    // Yaw   = degrees CLOCKWISE from arbitrary reference (- euler down->up axis)
    // Pitch = degrees UP from plane (- euler right->left axis)
    // Roll  = degrees rolling to the right (euler back->front axis)
    yaw : float;
    pitch : float;
    roll : float;
}
// YPR is a stupid way of doing things, at some point, every YPR will be
// replaced with this:
struct quat
{
    q0 : float; // imaginary x
    q1 : float; // imaginary y
    q2 : float; // imaginary z
    q3 : float; // real part
}
struct floatXYZ
{
    eq x : float;
    eq y : float;
    eq z : float;
}
struct floatXY
{
    eq x : float;
    eq y : float;
}
struct LatLongAlt
{
    // note that the reference geoid, and how the latitude/longitude/altitude
    // parameters are interpreted are actually defined by the base class.
    // Currently there is only one base class: WGS84Coord, which uses these
    // definitions:
    eq latitude : double;  // degrees north
    eq longitude : double; // degrees east
    eq altitude : float;   // metres up
}
struct ColourBase
{
    eq cmp type : ColourType;
    eq cmp values : float[4];
}
struct NodeInput
{
    eq node : int32;
    eq input : string;
    eq subType : int32;
}
struct LocalNodeInput
{
    eq cmp input : string;
    eq cmp subType : int32;
    // NB: schedType is excluded from comparison and equality testing
    schedType : InputSchedType;
    compatibleSubTypes : list<int32>;
}
struct NodeOutput
{
    eq node : int32;
    eq output : string;
    eq type : OutputType;
    eq subType : int32;
}
struct LocalNodeOutput
{
    eq cmp output : string;
    eq cmp type : OutputType;
    eq cmp subType : int32;
}
struct NodeInputArc
{
    eq input : string;
    eq src : NodeOutput;
}
struct NodeOutputArc
{
    eq dst : NodeInput;
    eq output : string;
}
struct TimeStamp <utility/time.h>
{
    // TODO: either use a base other than 1970, or more bits to store the
    // seconds
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
    // number of bins = round((rangeEnd-rangeStart) / rangeConversion)
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
    //values : map<MotorID, float>;
    prop: float;
    hbow: float;
    vbow: float;
    hstern: float;
    vstern : float;
}
struct Line
{
    eq centre : floatXY;
    eq angle : float;
    eq length : float;
    eq width : float;
}
struct Circle
{
    eq centre : floatXY;
    eq radius : float;
}
struct Ellipse
{
    eq centre : floatXY;
    eq majorRadius : float;
    eq minorRadius : float;
    eq angle : float; // radians, angle of major axis from x axis
}
struct Corner
{
    eq centre : floatXY;
    eq size : float;
    eq angle : float;
    eq response : float;
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
    eq pt : floatXY;
    eq size : float;
    eq angle : float;
    eq response : float;
    eq octave : float;
    eq class_id : float;
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
struct UID <utility/uid.h>
{
    eq cmp host : uint32;   // unique (constant) per-host, eg IP address / end of MAC
    eq cmp sensor : int32; // unique (constant) per-sensor
    eq cmp seq1 : int32;   // 
    eq cmp seq2 : int32;   // sequence per-sensor
}
enum BoundedFloatType : int8 {
    Clamps, Wraps
}
struct BoundedFloatBase
{
    eq value : float;
    eq min : float;
    eq max : float;
    eq type : BoundedFloatType;
}
struct ImageRegion
{
    eq top_left : floatXY;
    eq size     : floatXY;
}
struct Range
{
    eq min : float;
    eq max : float;
}
struct DynamicEnum
{
    eq value : string;
    eq possible : list<string>;
}

//fmt_ver should be bumped if image serialisation format changes
class BaseImage : <common/msg_classes/base_image.h> fmt_ver 1
BoundedFloatBase subclass BoundedFloat : <common/msg_classes/bounded_float.h>
LatLongAlt subclass WGS84Coord : <common/msg_classes/wgs84_coord.h>
floatXYZ subclass NorthEastDepth : <common/msg_classes/north_east_depth.h>
ColourBase subclass Colour : <common/msg_classes/colour.h>

enum SequenceType : int8 {
    Points, Path, Area
}

struct LocationSequence
{
    eq locations : list<WGS84Coord>;
    eq type : SequenceType;
}

variant ParamValue
{
    int32, float, string, bool,
    list<Corner>, list<Line>, list<Circle>, list<float>,
    list<KeyPoint>,
    BoundedFloat,
    list<int32>,
    ImageRegion,
    Range,
    floatXY,
    DynamicEnum,
    list<floatXY>,
    list<Ellipse>,
    LocationSequence,
    Colour,
    list<Colour>,
}

//these are zmq messaging specific
group zmq_internal
{ 
    // These ids are reserved for internal messaging system and should not be sent
    //  or changed without changing related zeromq code
    message NodeIDMarker : 223
    {
    }
    message DaemonIDMarker : 224
    {
    }

    //these message ids aren't special like the 2 above
    message DaemonConnections : 225
    {
        connect_strings : list<string>;
    }
    message DaemonConnected : 226
    {
        daemon_id: uint32;
    }
    message DaemonAnnounce : 227 
    {
        daemon_id: uint32;
        connect_string : string;
    }
    message SubscriptionConfirmMarker : 228
    {
    }
}

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
    
    // redherring only
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
    // Used by simulation to convey simulated orientation information to
    // control
    message State : 81
    {
        // Yaw   = degrees CLOCKWISE from arbitrary reference (- euler down->up axis)
        // Pitch = degrees UP from plane (- euler right->left axis)
        // Roll  = degrees rolling to the right (euler back->front axis)
        orientation : floatYPR;
    }
}

group telemetry
{
    message Telemetry : 3
    {
        // Yaw   = degrees CLOCKWISE from arbitrary reference (- euler down->up axis)
        // Pitch = degrees UP from plane (- euler right->left axis)
        // Roll  = degrees rolling to the right (euler back->front axis)
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
        location : WGS84Coord;
        speed : floatXYZ;
    }
    message GPSLocation : 89
    {
        location : WGS84Coord;
        track : float;
        speed: float;
        climb: float;
    }
}
group image
{
    message Image : 4
    {
        source : CameraID;
        lazy image : BaseImage;
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

    message SetPipeline : 15
    {
        pipelineName    : string;
        lazy nodeTypes  : map<int32, NodeType>;
        lazy nodeConnections : map<int32, map<LocalNodeInput, NodeOutput> >;
        lazy nodeParams : map<int32, map<LocalNodeInput, ParamValue> >;
    }

    message SetPipelineState : 16
    {
        pipelineName : string;
        pipelineState : PipelineState;
    }

    message SetNodeParameter : 8
    {
        pipelineName : string;
        nodeId : int32;
        paramId : string;
        value : ParamValue;
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

    message Ellipses : 137
    {
        name : string;
        ellipses : list<Ellipse>;
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

    message Points : 136
    {
        name : string;
        points : list<floatXY>;
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

    message Float : 138
    {
        name : string;
        value : float;
    }
}

group can_control
{
    message MotorControl : 400
    {
        prop : int8;
        hbow : int8;
        vbow : int8;
        hstern : int8;
        vstern : int8;
    }

    message LightControl : 150
    {
        lightId: LightID;
        intensity : uint8;     // 0 (off) -> 255 (max)
        duty_cycle : uint8;    // 0 (off) -> 255 (always on)
        cycle_period : uint16; // milliseconds
    }

    message PowerControl : 401
    {
        channel : PowerChannel;
        command : PowerControl;
    }
    
    // reinstated for compatibility
    message CuttingDevice : 151
    {
        strength : uint8;
    }
}

group can_status
{

    message BatteryStatus : 404
    {
        pack_id : uint16; // 64k enough?
        pack_charge  : uint16; // mV
        pack_voltage : uint16; // mV
        pack_location : BatteryLocation;
    }

    // lots more info is available from the bms...

    message PowerStatus : 153
    {
        // values should be averaged over the most recent time-period (however
        // often messages are sent)
        channel : PowerChannel;
        voltage : int16; // mV
        current : int16; // mA
        power   : int16; // mW
        state   : bool;  // 1 = on, 0 = off
    }
   
    message Alive : 40
    {
        board  : BoardID;
        status : PCBStatus;
        info   : string;
    }

    message Pressure : 50
    {
        fore : uint16; // arbitrary calibration (red herring pressure sensors)
        aft  : uint16; // arbitrary calibration (red herring pressure sensors)
    }
    
    // Barracuda has independent and asynchronous pressure sensors...
    message ForePressure : 51
    {
        pressure : uint16; // millibars (1000 ~ 1 atmosphere)
    }

    message AftPressure : 52
    {
        pressure : uint16; // millibars (1000 ~ 1 atmosphere)
    }

    message Temperature : 403
    {
        location : Component;
    }

    // reinstated for reherring compatibility
    message RedHerringBatteryStatus : 152
     {
         voltage : uint16;
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

    message ScriptResponse : 103
    {
        response : ScriptResponse;
    }


    message MotorState : 101
    {
        motorId : MotorID;
        speed : int8;
    }

    message Graphable : 104
    {
        name : string;
        value : float;
    }

    message RemoteControlsAlive : 105
    {
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
        params : map<LocalNodeInput, ParamValue>;
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
        params : map<LocalNodeInput, ParamValue>;
    }

    message GraphDescription : 118
    {
        pipelineName : string;
        lazy nodeTypes : map<int32, NodeType>;
        lazy nodeInputs : map<int32, map<LocalNodeInput, NodeOutput> >;
        lazy nodeOutputs : map<int32, map<LocalNodeOutput, list<NodeInput> > >;
        lazy nodeParams : map<int32, map<LocalNodeInput, ParamValue> >;
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
        throughput : float; // bits/second
        frequency : float; // Hz
        timeTaken : float; // milliseconds
        timeRatio : float;
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
        lazy image : BaseImage;
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
    message AIKeepAlive : 199
    {
        managerId : string;
    }
}
group guiai
{
    message AddTask : 202
    {
        taskType : string;
    }
    message RemoveTask : 203
    {
        taskId : string;
    }
    message SetTaskState : 204
    {
        taskId : string;
        conditionIds : list<string>;
        taskOptions : map<string, ParamValue>;
        scriptOptions : map<string, ParamValue>;
    }
    message TaskRemoved : 205
    {
        taskId : string;
    }
    message TaskState : 206
    {
        taskId : string;
        conditionIds : list<string>;
        taskOptions : map<string, ParamValue>;
        dynamicScriptOptions : map<string, ParamValue>;
        staticScriptOptions : map<string, ParamValue>;
        isCurrentlyRunning : bool;
        pipelineIds : list<string>;
    }
    message ScriptState : 207
    {
        taskId : string;
        debugValues : map<string, ParamValue>;
    }
    message AddCondition : 208
    {
        conditionType : string;
    }
    message RemoveCondition : 209
    {
        conditionId : string;
    }
    message SetConditionState : 210
    {
        conditionId : string;
        conditionOptions : map<string, ParamValue>;
    }
    message ConditionRemoved : 211
    {
        conditionId : string;
    }
    message ConditionState : 212
    {
        conditionId : string;
        conditionOptions : map<string, ParamValue>;
        debugValues : map<string, ParamValue>;
        pipelineIds : list<string>;
        conditionValue: bool;
    }
    message TaskTypes : 213
    {
        typeNames : list<string>;
    }
    message ConditionTypes : 214
    {
        conditionTypes : list<string>;
    }
    message RequestAIState : 215
    {
    }
    message ScriptControl : 216
    {
        taskId : string;
        command : ScriptCommand;
    }
    message AIControl : 217
    {
        command : AICommand;
    }
}

group simulation
{
    message SimPosition : 800
    {
        location : WGS84Coord;

        // Yaw   = degrees CLOCKWISE from arbitrary reference (- euler down->up axis)
        // Pitch = degrees UP from plane (- euler right->left axis)
        // Roll  = degrees rolling to the right (euler back->front axis)
        orientation : floatYPR;

        quaternion : quat;

        speed : floatXYZ; // metres per second, some arbitrary local coordinate system: TODO: fix this!
    }
}
