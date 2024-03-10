package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
This class handels the odometry for the tracktracer tool.

evertything is either in radiance or mm
 */

public class Odometry implements Runnable {

    public DcMotor encoderX1, encoderX2, encoderY;
    public double xEncoderOdset, yEncoderOfset, OdoWheelDiam, OdoGearing, ticksPerRot;

    // variables for robot orientation
    public double RobotPositionX, RobotPositionY;
    public double prevOrient;

    // variables for encoder deltas
    public  double deltaX1, deltaX2, deltaY;
    public double oldX1, oldX2, oldY;

    public static boolean StopRequested;

    private Thread t;

    public Odometry(DcMotor encoderX1, DcMotor encoderX2, DcMotor encoderY, double xEncoderOfset, double yEncoderOfset, double OdoWheelDiam, double OdemetryGearing, double ticksPerRot)
    {
        this.encoderX1 = encoderX1;
        this.encoderX2 = encoderX2;
        this.encoderY = encoderY;
        this.xEncoderOdset = xEncoderOfset;
        this.yEncoderOfset = yEncoderOfset;
        this.OdoWheelDiam =OdoWheelDiam;
        this.OdoGearing = OdemetryGearing;
        this.ticksPerRot = ticksPerRot;

        this.t = new Thread(this, "Odometry Thread");
        this.t.start();

    }

    void getEncoderDeltas()
    {   // function for getting the delta values from the encoders used for odometry
        double currentX1 = encoderX1.getCurrentPosition() * Math.PI * OdoWheelDiam * OdoGearing / ticksPerRot;
        deltaX1 = currentX1 - oldX1;
        oldX1 = currentX1;

        double currentX2 = -encoderX2.getCurrentPosition() * Math.PI * OdoWheelDiam * OdoGearing / ticksPerRot;
        deltaX2 = currentX2 - oldX2;
        oldX2 = currentX2;

        double currentY = encoderY.getCurrentPosition()  * Math.PI * OdoWheelDiam * OdoGearing / ticksPerRot;
        deltaY = currentY - oldY;
        oldY = currentY;
    }

    public double getOrientation()
    {
        //Function for getting the robot orientation using the gyro

//        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS );
        //      double heading =angles.firstAngle;

        double heading = (((encoderX2.getCurrentPosition() * Math.PI * OdoWheelDiam * OdoGearing / ticksPerRot)  + (encoderX1.getCurrentPosition() * Math.PI * OdoWheelDiam * OdoGearing / ticksPerRot))) / ( 2 * xEncoderOdset);


        return heading;
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
        double dRoboty = deltaY - (yEncoderOfset * getDeltaOrientation(getOrientation()));

        //transform robot Pose to field Pose
        double dFieldX = dRobotx * Math.cos(getOrientation()) - -dRoboty * Math.sin(getOrientation());
        double dFieldY = dRobotx * Math.sin(getOrientation()) + dRoboty * Math.cos(getOrientation());

        //add delta values to the coordinates
        RobotPositionX += dFieldX;
        RobotPositionY += dFieldY;

    }


    public void run()
    {
        try{
            //code to init the odo loop

            while(!StopRequested)
            {
                // looping code here
                getPosition();
                Thread.sleep(50); //sleeps thread to save cpu usage
            }
            Thread.interrupted();
        }
        catch (InterruptedException e)
        {
            throw new Error("The odomery thread is interupted");
        }
    }
}
