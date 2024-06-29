// This is a class for demonstrations
package org.firstinspires.ftc.teamcode;



import static org.firstinspires.ftc.teamcode.classes.Hardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.Hardware;

@Disabled
public class DemoClass {

    static Hardware RobotHardware = new Hardware();

    private static final double EncoderTicksPerRevolution = 560; //TODO define values
    private static final double WheelDiameter = 8;

    public static void initHardware(HardwareMap hw)
    {
        RobotHardware.StartHardware(hw);
    }
    private static double getHeading()
    { //TODO
        //Orientation orien= imu.getRobotOrientation();
       // return orien.firstAngle;
        return 0;
    }
    public static void DriveForward(double speed, double distance)
    {
        //TODO check if the code is correct



        double target = distance / (Math.PI * WheelDiameter) * EncoderTicksPerRevolution;

        lback.setTargetPosition((int)target);
        rback.setTargetPosition((int)target);
        lfront.setTargetPosition((int)target);
        rfront.setTargetPosition((int)target);

        lfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfront.setPower(speed);
        rback.setPower(speed);
        lfront.setPower(speed);
        lback.setPower(speed);

        while(lback.isBusy() || rback.isBusy() || lfront.isBusy() || rfront.isBusy())
        {

        }

        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




    }

    //        double frontLeftPower = (-Y1 + X1 + X2) / denominator;
    //        double backLeftPower = (-Y1 - X1 + X2) / denominator;
    //        double frontRightPower = (-Y1 - X1 - X2) / denominator;
    //        double backRightPower = (-Y1 + X1 - X2) / denominator;


    public static void Strave(double speed, double distance)
    {
        double target = distance / (Math.PI * WheelDiameter) * EncoderTicksPerRevolution;

        lback.setTargetPosition((int)-target);
        rback.setTargetPosition((int)target);
        lfront.setTargetPosition((int)target);
        rfront.setTargetPosition((int)-target);

        lfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfront.setPower(speed);
        rback.setPower(speed);
        lfront.setPower(speed);
        lback.setPower(speed);

        while(lback.isBusy() || rback.isBusy() || lfront.isBusy() || rfront.isBusy())
        {

        }

        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void Turn(double speed, double distance)
    {
        double target = distance / (Math.PI * WheelDiameter) * EncoderTicksPerRevolution;

        lback.setTargetPosition((int)target);
        rback.setTargetPosition((int)-target);
        lfront.setTargetPosition((int)target);
        rfront.setTargetPosition((int)-target);

        lfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rfront.setPower(speed);
        rback.setPower(speed);
        lfront.setPower(speed);
        lback.setPower(speed);

        while(lback.isBusy() || rback.isBusy() || lfront.isBusy() || rfront.isBusy())
        {

        }

        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
