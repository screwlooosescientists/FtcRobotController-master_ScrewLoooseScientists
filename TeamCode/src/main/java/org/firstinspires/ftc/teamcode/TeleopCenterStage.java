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
import org.firstinspires.ftc.teamcode.classes.extra.Node;
import org.firstinspires.ftc.teamcode.classes.extra.PID;

import static org.firstinspires.ftc.teamcode.classes.Hardware.*;

@TeleOp(name = "TeleOp :)", group = "Centerstage")
public class TeleopCenterStage extends LinearOpMode {

    Hardware RobotHardware = new Hardware();
    TouchSensor none;
    public Drivetrain CenterstageDriveTrain;
    public Drivetrain Powerplay;
    public Lift Slides;
    public Lift Arm;
    public PID ArmPID;
    public Lift Slider;
    public Intake intake;

    public ElapsedTime Runtime = new ElapsedTime();

    float x1, y1, x2;

    double arm_Target;
    boolean state = false;

    // todo: write your code here
    @Override
    public void runOpMode()
    {

        // hardware map reference
        RobotHardware.StartHardware(hardwareMap);

        // odo specs:  5.7f ,1,8192f,20.9f,7.6f
        CenterstageDriveTrain = new Drivetrain(imu, lfront, lback, rfront, rback);
        Arm = new Lift(armMotor, Lift.LiftType.SinlejointedArm, 100, 32, 0, 0.00755190904, true, 1, ArmLimit);
        Slider = new Lift(SliderMotor, Lift.LiftType.LinearSlides, 100, 32, 0.0025, 0, false, 1, SliderLimit);
        intake = new Intake(IntakeMotor, KlapServo, BakjeKlep,  PixelDetector);

        ArmPID = new PID(-1, -0.001, -0, 0, Runtime.seconds());

        telemetry.addData("status", "waiting for start");
        telemetry.update();

        CenterstageDriveTrain.Init();
        //Arm.Init();
       //Arm.CalibrateHome();
        intake.Init();
        PlaneRelease.setPosition(1);
        waitForStart();
        Runtime.reset();
        KlapServo.setPosition(0.55);
        Slider.Init();
        while(opModeIsActive())
        {
            x1 = gamepad1.left_stick_x;
            y1 = gamepad1.left_stick_y;
            x2 = gamepad1.right_stick_x;

            //CenterstageDriveTrain.DriveFieldCenter(x1, y1, x2);

            if(gamepad1.right_trigger> 0.3)
            CenterstageDriveTrain.DriveRobotCenter(x1/3, y1/3, x2/3);
            else {    CenterstageDriveTrain.DriveRobotCenter(x1, y1, x2);

            }

            if(!SliderLimit.isPressed())
            {
                Slider.MoveLift(gamepad2.right_stick_y);
            }
            else
                if(gamepad2.right_stick_y > 0)
                {
                    Slider.MoveLift(gamepad2.right_stick_y);
                }
                else
                    Slider.MoveLift(0);

            Slider.MoveLift(gamepad2.right_stick_y);

            if(gamepad2.left_stick_y != 0)
            {
                Arm.MoveLift(gamepad2.left_stick_y * 0.7f);
                arm_Target = Arm.getAngle();
            }
           else{


              Arm.MoveLift((float)ArmPID.pidValue(Arm.getAngle(),arm_Target, Runtime.seconds()));
                /*telemetry.addData("elapsed time", Runtime.seconds());
                telemetry.addData("pid value", ArmPID.pidValue(Arm.getAngle(),arm_Target, Runtime.seconds()));
                telemetry.addData("p", (float)ArmPID.P(arm_Target, Arm.getAngle()));
                telemetry.addData("i", (float)ArmPID.I(arm_Target, Arm.getAngle(), Runtime.seconds()));
                telemetry.addData("d", (float)ArmPID.D(arm_Target, Arm.getAngle(), Runtime.seconds()));
                telemetry.addData("dt", ArmPID.DeltaTime(Runtime.seconds()));
                */

            }
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

  //Test telemetry----------------------------------------------------------------------------




 //Telemetry---------------------------------------------------------------------------------
            telemetry.addData("status", "running" );
            //telemetry.addData("pixels: ", intake.TwoPixels);
            //telemetry.addData("EncoderPosX1:", CenterstageDriveTrain.encoderX1.getCurrentPosition());
           // telemetry.addData("EncoderPosX2:", CenterstageDriveTrain.encoderX2.getCurrentPosition());
            //telemetry.addData("Position: ", CenterstageDriveTrain.RobotPositionX + ", " + CenterstageDriveTrain.RobotPositionY);
           // telemetry.addData("Orientation", (CenterstageDriveTrain.getOrientation() / Math.PI));
            //telemetry.addData("is pressed: ", ArmLimit.isPressed());
            telemetry.addData("Arm angle", (Arm.getAngle() / Math.PI) * 180);
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



