package TTRobotSide.Util.Localization;


import TTRobotSide.Util.Localization.Position;
import TTRobotSide.Util.Localization.Angle.angleType;

public class Pose {
    
    public Position position = new Position(0, 0);    //The x and why components of the pose, where x is forwards.
    public Angle Alpha = new Angle(0, angleType.UNDEFINED);    //The angle of the pose, 0 degrees == the y axis.

    public Pose(Position position, Angle Alpha)
    {
        this.position = position;
        this.Alpha = Alpha;
    }

}
