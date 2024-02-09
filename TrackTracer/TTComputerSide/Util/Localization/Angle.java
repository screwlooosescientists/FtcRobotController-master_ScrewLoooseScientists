package TTRobotSide.Util.Localization;

public class Angle {
   
    public double angle;
    public angleType type = angleType.UNDEFINED;

    public enum angleType // the enum for getting the angle angleType
    {
        DEGREES,
        RADIANS,
        UNDEFINED
    }

    public Angle(double angle, angleType type)
    {
        this.angle = angle;
        this.type = type;
        if (type == angleType.UNDEFINED) {
            throw new Error("Angle type must not be undefined");
        }
    }


    public double convertToAngleType(double value, angleType originType, angleType newType)
    {
        double val = 0;

        if(originType == angleType.DEGREES && newType == angleType.RADIANS) {
            val = val * Math.PI / 180d;
            type = angleType.RADIANS;
        }
        else
        if(originType == angleType.RADIANS && newType == angleType.DEGREES)
        {
            val = val * 180 / Math.PI;
            type = angleType.DEGREES;
        }
        else
        if(originType == angleType.UNDEFINED)
        throw new Error("Origin type is Unknown and thus canot be conferted");
       
        return val;
    }

}
