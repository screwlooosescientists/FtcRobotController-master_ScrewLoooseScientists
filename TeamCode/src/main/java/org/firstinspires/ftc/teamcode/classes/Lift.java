package org.firstinspires.ftc.teamcode.classes;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;


public class Lift extends Robot{
    // hardware
    DcMotor liftMotor;
    TouchSensor LiftLimit;
    LiftType SortLift;

    //variables & properties
    double MaxHeight;
    double liftLength;
    double HeightPerEncoderTick;
    double AnglePerEncoderTick;
    double LiftAngle;
    double LastLiftAngle;
    float speed;
    double previousLiftHeight;
    public double previousTime;

    boolean IsPositive;

    // PID gains


    public Lift(DcMotor liftMotor, LiftType type, double MaxHeight, double liftLength, double HeightPerEncoderTick, double AnglePerEncoderTick, boolean IsPositive, float speed, TouchSensor LiftLimit)
    {
        this.SortLift = type;
        this.liftMotor = liftMotor;
        this.LiftLimit = LiftLimit;
        this.MaxHeight = MaxHeight;
        this.HeightPerEncoderTick = HeightPerEncoderTick;
        this.AnglePerEncoderTick = AnglePerEncoderTick;
        this.IsPositive = IsPositive;
        this.liftLength = liftLength;
        this.speed = speed;

    }


   public enum LiftType
    {
        DR4B,
        LinearSlides,
        SinlejointedArm,

        SinglejointedSLides
    }


    public void MoveToHeight(double TargetHeigth)
    {
        double value = 0;
        switch (SortLift)
        {
            case LinearSlides:
                value = (TargetHeigth - 30) / HeightPerEncoderTick;
            break;

            case SinlejointedArm:
                value = Math.asin(TargetHeigth/liftLength);
            break;

            case DR4B:
                value = 0;
            break;
        }



        if(!IsPositive)
        {
            value = -value;
        }

        liftMotor.setTargetPosition((int)(value));

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(speed);

    }

    public void MoveToAngle(double TargetAngle) {
        double value = TargetAngle;
        /*
        switch (SortLift) {
            case LinearSlides:

                break;

            case SinlejointedArm:
                value = TargetAngle;
                break;

            case DR4B:

                break;
        }

         */
        liftMotor.setTargetPosition((int)(value  / AnglePerEncoderTick));

        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setPower(speed);

    }
     public void  MoveLift(float speed)
    {
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        float value = speed;
        if(!IsPositive)
            value = -value;

        liftMotor.setPower(value);
    }

    public void CalibrateHome()
    {
        while(!LiftLimit.isPressed())
        {
            MoveLift(0.1f);
        }
        MoveLift(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double LiftSpeed(double milliseconds)
    {
        double deltaTime = milliseconds - previousTime;
        double deltaLiftHeight = CurrentHeigth() - previousLiftHeight;

        previousLiftHeight = CurrentHeigth();
        previousTime = milliseconds;
        return deltaLiftHeight / deltaTime;
    }

    public double CurrentHeigth() //Calculates the starting heigth of the lift
    {
       double value = 0;

            switch (SortLift) {
                case LinearSlides:
                    value = liftMotor.getCurrentPosition() * HeightPerEncoderTick;
                break;

                case SinlejointedArm:
                    value = liftLength * Math.sin(liftMotor.getCurrentPosition() * AnglePerEncoderTick);
                break;

                case DR4B:
                    value = 0;
                break;


            }

            if(IsPositive)
                value = -value;

        return value;

    }

    public double getAngle()
    {
        return -liftMotor.getCurrentPosition() * AnglePerEncoderTick;
    }

    public double GetDeltaAngle(double currentAngle)
    {
        double DeltaAngle;
        DeltaAngle = currentAngle - LiftAngle;
        LastLiftAngle = currentAngle;
        return DeltaAngle;
    }

    public float compensate(float alpha,float A_x,float A_y, float r)
    {
        //create B
        float[] B = new float[2];
        B[0] = r * (float)Math.cos(alpha);    // Calculate the x of B using goniometric functions
        B[1] = r * (float)Math.sin(alpha);    // Calculate the y of B using goniometric functions

        //calculate length s
        float DeltaX = B[0] - A_x;
        float DeltaY = B[1] - A_y;
        float s = (float) Math.sqrt(Math.pow(DeltaX, 2) + Math.pow(DeltaY, 2));

        //Find minimum s "d" by subtracting  length "p" from radius "r"
        float DeltaX2 = -A_x; // M(0,0) thus length from M to A is just negative A
        float DeltaY2 = -A_y;
        float p = (float) Math.sqrt(Math.pow(DeltaX2, 2) + Math.pow(DeltaY2, 2));
        float d = r - p;

        // Calculate compensation "q" by substracting d from s
        return s - d;
    }


    @Override
    public void Init() {    // makes the lift go down until it reaches the limit switch and resets encoders

       // while (!LiftLimit.isPressed())
       // {
            //MoveLift(0.1f);
        //}

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }
}
