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

enum MotorID { PROP = 1, HBOW = 2, VBOW = 4, HSTERN = 8, VSTERN = 16 }
enum CameraID { cam_forward = 1, cam_down = 2, cam_sonar = 4, cam_file = 5 }
enum NodeType {
    nt_invalid = 0,
    nt_copy = 1,
    nt_resize = 2,
    nt_file_input = 3,
    nt_file_output = 4,
    nt_local_display = 5,
    nt_max_num_node_types = 6,
    nt_camera_input = 7,
    nt_net_input = 8,
    nt_hough_linesp = 9,
    nt_canny = 10
}
enum NodeParameterType {
    pt_int32 = 0,
    pt_float = 1,
    pt_string = 2,
    pt_max_num_parameter_types = 3
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

    message NodeAdded : 6
    {
        nodeId : int32;
    }

    message RemoveNode : 7
    {
        nodeId : int32;
    }

    message ClearPipeline : 8
    {
    }

    message SetNodeParameter : 9
    {
        nodeId : int32;
        paramId : string;
        paramType : NodeParameterType;
        intValue : int32;
        floatValue : float;
        stringValue : string;
    }

    message GraphDescription : 10
    {
        unused : int8;
    }
}

