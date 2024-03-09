package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.classes.Hardware.*;


public class Intake extends Robot{
    // hardware
    public DcMotor IntakeMotor;

    public Servo KlapServo;
    public Servo PixelRelease;

    public ColorSensor PixelDetector;
    //variables
    public boolean TwoPixels;

    //Constructor
    public Intake(DcMotor IntakeMotor, Servo KlapServo, Servo PixelRelease, ColorSensor PixelDetector)
    {
       this.IntakeMotor = IntakeMotor;
       this.KlapServo = KlapServo;
       this.PixelRelease = PixelRelease;
       this.PixelDetector = PixelDetector;

    } 

    public void DriveIntake(float Speed)
    {
        IntakeMotor.setPower(Speed);
    }

    public void AutomateIntake(boolean Down)
    {


        if(PixelDetector.argb() > 80)
            TwoPixels = false;
        else
            TwoPixels = true;

        if(Down && TwoPixels)
        {

            IntakeMotor.setPower(1);
            KlapServo.setPosition(0.55);
        }
        else
        if(Down && TwoPixels == false)
        {
            IntakeMotor.setPower(1);
            KlapServo.setPosition(0.55);
        }
        else
        {
            IntakeMotor.setPower(0);
            KlapServo.setPosition(1);
        }

    }

    public void PixelReleaseControl(boolean ReleaseButtonState)
    {
        if (!ReleaseButtonState)
            PixelRelease.setPosition(1);
        else
            PixelRelease.setPosition(0.5);
    }



    public void Init()
    {


        KlapServo.setPosition(1);
    }
    
}
