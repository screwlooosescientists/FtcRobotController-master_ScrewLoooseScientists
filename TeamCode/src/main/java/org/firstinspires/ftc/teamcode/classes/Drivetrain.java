package org.firstinspires.ftc.teamcode.classes;

// imports
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.extra.Node;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static org.firstinspires.ftc.teamcode.classes.Hardware.*;

public class Drivetrain extends Robot  {

    // Properties of drivetrain
    public DcMotor Lfront, Lback, Rfront, Rback;    //the drive motors
    public IMU imu;
    public DcMotor encoderX1, encoderX2, encoderY;  // the encoders are declared as dc motors becouse they use that ports on the controll hub

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
    float deltaTime;
    float lastTime;

    //robot pose vars
    float lastDistance;
    float distance;
    public double prevOrient;

    public float wheelDiameter, GearRatio; // for the driven wheels

    public float  odometryDiameter, verticalEncoderOfset, horizontalEncoderOfset; // for odometry



    // variables for robot orientation
      public double RobotPositionX, RobotPositionY;

      // variables for encoder deltas
       public  double deltaX1, deltaX2, deltaY;
       public double oldX1, oldX2, oldY;

    // constructor
  public  Drivetrain(IMU imu, DcMotor Lfront, DcMotor Lback, DcMotor Rfront, DcMotor Rback, DcMotor encoderX1, DcMotor encoderX2, DcMotor encoderY, float WheelDiameter, float GearRatio,float odometryDiameter, float verticalEncoderOfset, float horizontalEncoderOfset){
        this.wheelDiameter = WheelDiameter;
        this.GearRatio = GearRatio;
        this.odometryDiameter = odometryDiameter;
        this.verticalEncoderOfset = verticalEncoderOfset;
        this.horizontalEncoderOfset = horizontalEncoderOfset;
        this.Lfront = Lfront;
        this.Lback = Lback;
        this.Rfront = Rfront;
        this.Rback = Rback;
        this.imu = imu;
        this.encoderX1 = encoderX1;
        this.encoderX2 = encoderX2;
        this.encoderY = encoderY;

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
        double heading = getOrientation();
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
    }

    public void followPath(Node[] path, float pathAcuracy, float orientation)
    {
/*
        for(int i = 0; i < path.lengt; i++)
        {
            while(distance > pathAcuracy)   //TODO (add al conditions such as orientation and actions in between points) drive to point until conditionts met to go to the next point
            {
                DriveToPoint(path[index]);
            }
            
            
        }
*/
    }

    void getEncoderDeltas()
    {   // function for getting the delta values from the encoders used for odometry
        double currentX1 = encoderX1.getCurrentPosition() * Math.PI * wheelDiameter / odometryDiameter;
        deltaX1 = currentX1 - oldX1;
        oldX1 = currentX1;

        double currentX2 = -encoderX2.getCurrentPosition() * Math.PI * wheelDiameter / odometryDiameter;
        deltaX2 = currentX2 - oldX2;
        oldX2 = currentX2;

        double currentY = encoderY.getCurrentPosition()  * Math.PI * wheelDiameter / odometryDiameter;
        deltaY = currentY - oldY;
        oldY = currentY;
    }

    public double getOrientation()
    {
        //Function for getting the robot orientation using the gyro
        //TODO get robot orientation by using the encoder calculations.
//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS );
  //      double heading =angles.firstAngle;

        robotOrientation = imu.getRobotYawPitchRollAngles();


       return robotOrientation.getYaw(AngleUnit.RADIANS);
    }

    public double getDeltaOrientation(double curretnOrientation)
    {

        double deltaOrien = curretnOrientation - prevOrient;
        prevOrient = curretnOrientation;
        return  deltaOrien;
    }

    public void getPosition()
    {
        getEncoderDeltas(); // cals the function to retrieve new encoder data

        //get delta robot position
        double dRobotx = -(deltaX1 + deltaX2) / 2;
        double dRoboty = deltaY - (horizontalEncoderOfset * getDeltaOrientation(getOrientation()));

        //transform robot Pose to field Pose
        double dFieldX = dRobotx * Math.cos(getOrientation()) - -dRoboty * Math.sin(getOrientation());
        double dFieldY = dRobotx * Math.sin(getOrientation()) + dRoboty * Math.cos(getOrientation());

        //add delta values to the coordinates
        RobotPositionX += dFieldX;
        RobotPositionY += dFieldY;

    }

    @Override
    public void Init() { //TODO make a init function, robot calib etc

      imu.resetYaw();

    }
}
