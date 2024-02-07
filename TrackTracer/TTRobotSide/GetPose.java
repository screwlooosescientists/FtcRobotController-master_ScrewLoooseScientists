package TTRobotSide;

import TTRobotSide.Util.Localization.*;
import TTRobotSide.Util.Localization.Angle.angleType;

public class GetPose {

   
    Pose pose;

    public Angle getAngle() //TODO write the code to get the position using encoders
    {
        return new Angle(0, angleType.UNDEFINED);
    }

    public Position getPosition() //TODO write the code to get the pos 
    {
        return new Position(0, 0);
    }

    public Pose getPose()
    {
        pose.position = getPosition();
        pose.Alpha = getAngle();
        return pose;
    }
}
