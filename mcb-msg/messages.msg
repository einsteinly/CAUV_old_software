enum DebugType : int8 {
    Info = 0,
    Debug = 1,
    Warning = 2,
    Error = 3
}
enum MotorID : int8 { Prop = 1, HBow = 2, VBow = 4, HStern = 8, VStern = 16 }
enum LightID: int8 { Forward = 1, Down = 2 }

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

	message ResetMCB : 85
	{
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
}
