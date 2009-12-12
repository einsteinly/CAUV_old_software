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

enum MotorID { PROP = 1, HBOW = 2, VBOW = 4, HSTERN = 8, VSTERN = 16 }
enum CameraID { FORWARD = 1, DOWN = 2, SONAR = 4 }

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

