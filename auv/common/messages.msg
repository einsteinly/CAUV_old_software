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
    dst : string;
    src : NodeOutput;
}
struct NodeOutputArc
{
    dst : NodeInput;
    src : string;
}

enum MotorID { PROP = 1, HBOW = 2, VBOW = 4, HSTERN = 8, VSTERN = 16 }
enum CameraID { FORWARD = 1, DOWN = 2, SONAR = 4 }
enum NodeType {
    nt_invalid = 0,
    nt_copy = 1,
    nt_resize = 2,
    nt_file_input = 3,
    nt_file_output = 4,
    nt_local_display = 5,
    nt_max_num_node_types = 255
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
	    type : int8;
        image : list<byte>;
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

    message SetNodeParameter : 8
    {
        nodeId : int32;
    }

    message GraphDescription : 9
    {
        unused : int8;
    }
}

