package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.classes.Drivetrain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.classes.Hardware;
import org.firstinspires.ftc.teamcode.classes.Intake;
import org.firstinspires.ftc.teamcode.classes.Lift;

import static org.firstinspires.ftc.teamcode.classes.Hardware.*;

@TeleOp(name = "TeleOp :)", group = "Centerstage")
public class TeleopCenterStage extends LinearOpMode {

    Hardware RobotHardware = new Hardware();
    TouchSensor none;
    public Drivetrain CenterstageDriveTrain;
    public Lift Slides;
    public Lift Arm;
    public Lift Slider;
    public Intake intake;

    public ElapsedTime Runtime = new ElapsedTime();

    float x1, y1, x2;
    boolean state = false;

    // todo: write your code here
    @Override
    public void runOpMode()
    {

        // hardware map reference
        RobotHardware.StartHardware(hardwareMap);

        CenterstageDriveTrain = new Drivetrain(imu, lfront, lback, rfront, rback, rfront , rback, lfront, 1,1,5.8f,1,1); // TODO add odometry pod stuf
        Arm = new Lift(armMotor, Lift.LiftType.SinlejointedArm, 100, 32, 0, 0.00755190904, true, 1, ArmLimit);
        Slider = new Lift(SliderMotor, Lift.LiftType.LinearSlides, 100, 32, 0.0025, 0, false, 1, ArmLimit);
        intake = new Intake(IntakeMotor, KlapServo, BakjeKlep,  PixelDetector);


        telemetry.addData("status", "waiting for start");
        telemetry.update();

        CenterstageDriveTrain.Init();
        Arm.Init();
       // Arm.CalibrateHome();
        intake.Init();
        PlaneRelease.setPosition(1);
        waitForStart();
        Runtime.reset();
        KlapServo.setPosition(0.26);
        while(opModeIsActive())
        {
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;

            //CenterstageDriveTrain.DriveFieldCenter(x1, y1, x2);

            if(gamepad1.right_trigger> 0.3)
            CenterstageDriveTrain.DriveRobotCenter(x1/3, y1/3, x2/3);
            else
                CenterstageDriveTrain.DriveRobotCenter(x1, y1, x2);

            Slider.MoveLift(gamepad2.right_stick_y);
            Arm.MoveLift(gamepad2.left_stick_y);

//reseting gyro----------------------------------------------------------------------------------------
            if (gamepad1.dpad_left && gamepad1.b)
                CenterstageDriveTrain.imu.resetYaw();


// Changing the position of the intake------------------------------------------------------------------

            if(gamepad2.dpad_up)
            {
               state = false;
            }
            else
                if(gamepad2.dpad_down)
                {
                    state = true;
                }

            if(state == false)
            {
                intake.AutomateIntake(false);
            }
            else
            if(state)
            {
                intake.AutomateIntake(true);
            }

  //PixelRelease---------------------------------------------------------------------------

            intake.PixelReleaseControl(gamepad2.a);

  //Plane firing --------------------------------------------------------------------------

            if(gamepad1.left_bumper && gamepad1.left_trigger == 1 && gamepad1.right_bumper && gamepad1.right_trigger == 1)
                PlaneRelease.setPosition(0);


 //Telemetry---------------------------------------------------------------------------------
            telemetry.addData("status", "running" );
            telemetry.addData("pixels: ", intake.TwoPixels);
            telemetry.addData("Orientation", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("is pressed: ", ArmLimit.isPressed());
            telemetry.update();
        }
    }

    public void MoveLiftToPoint(double x, double y)
    {
        double aplha; // lift angle in radians
        double r;     // lift radius

        r = Math.sqrt((x * x) + (y * y));
        aplha = Math.asin(y / r);

        Arm.MoveToAngle(aplha);
        Slider.MoveToHeight(r);


    }

}


