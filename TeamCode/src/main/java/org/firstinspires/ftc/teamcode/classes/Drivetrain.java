package org.firstinspires.ftc.teamcode.classes;

// imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.extra.Node;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.classes.extra.PID;

import static org.firstinspires.ftc.teamcode.classes.Hardware.*;

public class Drivetrain extends Robot  {

    // Properties of drivetrain
    public DcMotor Lfront, Lback, Rfront, Rback;    //the drive motors
    public IMU imu;


    // pid gains
   public static double driveKp = 0;
   public static double driveKi = 0;
   public static double driveKd = 0;

   float P = 0, I = 0, D = 0;
    public float Fl = 0, Fr = 0, Bl = 0, Br = 0;

   //imu angles
  public YawPitchRollAngles robotOrientation;

   // timer
    ElapsedTime timer = new ElapsedTime();

    //robot pose vars
    float lastDistance;
    float distance;


    public float wheelDiameter, GearRatio; // for the driven wheels


    public PID drivePIDX = new PID(0, 0, 0, 0, timer.seconds());
    public PID drivePIDY = new PID(0, 0, 0, 0, timer.seconds());
    public PID drivePIDAngle = new PID(0, 0, 0, 0, timer.seconds());


    // variables for robot orientation
      public double RobotPositionX, RobotPositionY, RobotHeading;

      // variables for encoder deltas
       public  double deltaX1, deltaX2, deltaY;
       public double oldX1, oldX2, oldY;

    // constructor
  public  Drivetrain(IMU imu, DcMotor Lfront, DcMotor Lback, DcMotor Rfront, DcMotor Rback){
        this.Lfront = Lfront;
        this.Lback = Lback;
        this.Rfront = Rfront;
        this.Rback = Rback;
        this.imu = imu;
    }

    public void setRobotPose(double posX, double posY, double Heading)
    {
        this.RobotPositionX = posX;
        this.RobotPositionY = posY;
        this.RobotHeading = Heading;
    }


    //function for robot oriented drive
    public void DriveRobotCenter(float X1, float Y1, float X2 )
    {
        double denominator = Math.max(Math.abs(X1) + Math.abs(Y1) + Math.abs(X2), 1);
        double frontLeftPower = (-Y1 + X1 + X2) / denominator;
        double backLeftPower = (-Y1 - X1 + X2) / denominator;
        double frontRightPower = (-Y1 - X1 - X2) / denominator;
        double backRightPower = (-Y1 + X1 - X2) / denominator;

        Lfront.setPower(frontLeftPower);
        Lback.setPower(backLeftPower);
        Rfront.setPower(frontRightPower);
        Rback.setPower(backRightPower);
    }

    public void DriveFieldCenter(float X1, float Y1, float X2)
    {
        double heading = RobotHeading;
        double rotX = X1 * Math.cos(heading) - Y1 * Math.sin(heading);
        double rotY = X1 * Math.sin(heading) + Y1 * Math.cos(heading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(Y1) + Math.abs(X1) + Math.abs(X2), 1);
        double frontLeftPower = (-rotY + rotX + X2) / denominator;
        double backLeftPower = (-rotY - rotX + X2) / denominator;
        double frontRightPower = (-rotY - rotX - X2) / denominator;
        double backRightPower = (-rotY + rotX - X2) / denominator;

        Lfront.setPower(frontLeftPower);
        Lback.setPower(backLeftPower);
        Rfront.setPower(frontRightPower);
        Rback.setPower(backRightPower);
    }

    public void DriveToPoint(Node Target)
    {   // get data needed for calculations


       double x = drivePIDX.pidValue(RobotPositionX, Target.X, timer.time());
       double y = drivePIDY.pidValue(RobotPositionY, Target.Y, timer.time());
       double angle = drivePIDAngle.pidValue(RobotHeading, Target.TargetHeading, timer.time());

        double deltaX = RobotPositionX - Target.X;
        double deltaY = RobotPositionY - Target.Y;

        distance = (float)Math.sqrt(deltaX * deltaX + deltaY * deltaY);

       double xRot = (x * Math.cos(RobotHeading) - (y * Math.sin(RobotHeading)));
       double yRot = (x * Math.cos(RobotHeading) + (y * Math.sin(RobotHeading)));

        double frontLeftPower = (-xRot + yRot + angle);
        double backLeftPower = (-xRot - yRot + angle);
        double frontRightPower = (-xRot - yRot - angle);
        double backRightPower = (-xRot + yRot - angle);

        double  denominator = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.max( Math.abs(backRightPower), 1))));
        Lfront.setPower(frontLeftPower / denominator);
        Lback.setPower(backLeftPower / denominator);
        Rfront.setPower(frontRightPower / denominator);
        Rback.setPower(backRightPower / denominator);


        /*

        deltaTime = (float) timer.milliseconds() - lastTime;    // get delta time
        float deltaDistance = distance - lastDistance;          // gets the delta distance to the target Pose
        float deltaX = Target.X - (float) RobotPositionX;        //gets the X distance to the target Pose
        float deltaY = Target.Y - (float) RobotPositionY;        //gets the Y distance to the target Pose 
        float robotTheta =(float) getOrientation();              //gets the robot orientation
        float deltaThetha = Target.TargetHeading - robotTheta;   //gets the amount of degrees the robot needs to rotate to get to the target Pose

        //transform the directions the robot needs to drive to robot relative values
       float transformtDx = deltaX * (float) Math.cos( robotTheta) - deltaY *  (float) Math.sin(robotTheta); //TODO make sure that robotTheta may needs to be negative
       float transformtDy = deltaX * (float) Math.sin(robotTheta) + deltaY * (float) Math.cos(robotTheta);

       //set last distance to current distance and calculate new distance
       lastDistance = distance;
       distance = (float)Math.sqrt(deltaX * deltaX + deltaY * deltaY);

        // calculates the PID values for the scaling of the motorspeed to ensure that the robot obtains its target Pose acurately
        P = (float)driveKp * distance;
        I = I + (float)driveKi * deltaDistance + deltaTime;
        D = (float)driveKd * deltaDistance / deltaTime;

        // adds values to get unscaled robot values
        //TODO add a rotation function
         Fl = -(-transformtDy + transformtDx) * (P + I + D);  // Fl = front left
        Bl = -(-transformtDy - transformtDx) * (P + I + D); // Bl = back left
        Fr = -(-transformtDy - transformtDx) * (P + I + D);  // Fr = front right
        Br = (transformtDy + transformtDx) * (P + I + D);  // Br = bakc left
    
       //Normalizeses the motorvalues to stay between -1 and 1 and asign them to the motors
       float denominator = Math.max(Math.abs(Fl), Math.max(Math.abs(Bl), Math.max(Math.abs(Fr), Math.max( Math.abs(Br), 1f))));




       Lfront.setPower(Fl);
       Lback.setPower(Bl);
       Rfront.setPower(Fr);
       Rback.setPower(Br);

        //gets delta time
        lastTime = (float) timer.milliseconds();

         */



    }

    public void followPath(Node[] path, float pathAcuracy, float orientation)
    {

        for(int i = 0; i < path.length; i++)
        {
            while(distance > path[i].acuracy)   //TODO (add al conditions such as orientation and actions in between points) drive to point until conditionts met to go to the next point
            {

                if(path[i].HasCondition == true)
                {

                }
                DriveToPoint(path[i]);
            }
            
            
        }

    }


    @Override
    public void Init() { //TODO make a init function, robot calib etc

      imu.resetYaw();

    }
}
