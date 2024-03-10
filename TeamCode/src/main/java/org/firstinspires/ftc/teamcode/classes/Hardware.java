package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Hardware {

    private static HardwareMap hwmap = null;
    // create motors

  public static DcMotor lfront;
  public static DcMotor rfront;
  public static DcMotor lback;
  public static DcMotor rback;



  public static DcMotor armMotor;
  public static DcMotor SliderMotor;

  public static DcMotor IntakeMotor;


    // create servo
    public static Servo KlapServo;
    public static Servo BakjeKlep;
    public static Servo PlaneRelease;

    // create sensors
    public static IMU imu;
    public static TouchSensor ArmLimit;
    public static TouchSensor SliderLimit;
    public static ColorSensor PixelDetector;

    // create cameras and visual portal
    public static CameraName cam;
    public static VisionPortal vsPortal;
    public static TfodProcessor TfodProcessor;
    public static AprilTagProcessor AprilProcessor;
    // Additional variables

    public  Hardware()
    {

    }

  static  public void StartHardware(HardwareMap hardwareMap)
    {
        hwmap = hardwareMap;

        // connect motors
        lfront = hwmap.get(DcMotor.class, "Lfront"); //also the y encoder for odometry
        rfront = hwmap.get(DcMotor.class, "Rfront"); //also the x1 encoder for odometry
        lback = hwmap.get(DcMotor.class, "Lback");
        rback = hwmap.get(DcMotor.class, "Rback"); //also the x2 encoder for odometry
        armMotor = hardwareMap.get(DcMotor.class, "Arm");
        SliderMotor = hardwareMap.get(DcMotor.class, "Slider");
        IntakeMotor = hardwareMap.get(DcMotor.class, "Intake");


        // connect servos
        KlapServo = hwmap.get(Servo.class, "KlapServo");        //Expansion hub Servo Port 5
        BakjeKlep = hwmap.get(Servo.class, "BakjeKlep");        //Control hub Servo Port 5
        PlaneRelease = hwmap.get(Servo.class, "PlaneRelease");  //Expension hub Servo Port 0

        // connect sensors
        SliderLimit = hwmap.get(TouchSensor.class, "ArmLimit");
        ArmLimit = hwmap.get(TouchSensor.class, "SliderLimit");
        PixelDetector = hwmap.get(ColorSensor.class, "PixelDetector");

        //imu
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        //Cameras
        cam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));



        // Set motor Modes
        lfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rback.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rback.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SliderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lfront.setPower(0);
        rfront.setPower(0);
        lback.setPower(0);
        rback.setPower(0);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

}
