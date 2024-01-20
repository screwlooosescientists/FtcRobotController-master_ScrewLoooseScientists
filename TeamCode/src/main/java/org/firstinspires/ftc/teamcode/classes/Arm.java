package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends Robot{
   // hardware and variables
    public Servo ArmServ;
    float FrontBound, RearBound;

    //constructor
    public Arm(Servo ArmServ, float FrontBound, float RearBound)
    {
        this.ArmServ = ArmServ;
        this.FrontBound = FrontBound;
        this.RearBound = RearBound;
    }

    public void MoveArm(boolean FrontPosButton, boolean RearPosButton) // moves the rotating arm to either the front or the rear position
    {
        if(FrontPosButton)
            ArmServ.setPosition(FrontBound);

        if(RearPosButton)
            ArmServ.setPosition(RearBound);
    }

    public boolean ArmIsAtFront()
    {
        if(ArmServ.getPosition() == FrontBound)
            return true;
        else
            return false;
    }

    public void Init()
    {
        ArmServ.setPosition(RearBound);
    }
}