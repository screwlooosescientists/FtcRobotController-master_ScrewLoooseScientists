package TTRobotSide.SubClasses;

import TTRobotSide.Util.Localization.*;
import TTRobotSide.Util.Localization.Angle.angleType;

public class Odometry {

    //Variables
    Pose pose;  //The variable that stores the pose data

    double X1, X2, Y;
    double xOfset, yOfset, PodDiam, GearRatio, EncoderTicksPerRot;

    //Constructor
    public Odometry(double X1, double X2, double Y, double xOfset, double yOfset, double PodDiam, double GearRatio, double EncoderTicksPerRot)
    {
        this.X1 = X1;
        this.X2 = X2;
        this.Y = Y;
        this.xOfset = xOfset;
        this.yOfset = yOfset;
        this.PodDiam = PodDiam;
        this.GearRatio = GearRatio;
        this.EncoderTicksPerRot = EncoderTicksPerRot;
    }

    public double X1dist()
    {
        return X1 * PodDiam * GearRatio / EncoderTicksPerRot;
    }

    public double X2dist()
    {
        return X2 * PodDiam * GearRatio / EncoderTicksPerRot;
    }

    public double Ydist()
    {
        return Y * PodDiam * GearRatio / EncoderTicksPerRot;
    }

    public Angle getAngle() //TODO write the code to get the position using encoders
    {
        double rotDist = (X1dist() + X2dist()) / 2; //The distance the wheels make for rotation only, so the lenght of the circkle part.
        double val = rotDist / xOfset;              //Circle length devided by the (0.5 * diameter) gives the angle in radians
        return new Angle(val, angleType.RADIANS);
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
