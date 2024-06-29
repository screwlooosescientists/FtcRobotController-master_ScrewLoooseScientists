package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.DemoClass.DriveForward;
import static org.firstinspires.ftc.teamcode.DemoClass.Strave;
import static org.firstinspires.ftc.teamcode.DemoClass.Turn;
import static org.firstinspires.ftc.teamcode.DemoClass.initHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous(group = "Demo", name = "Autonomous")
public class OpMode extends LinearOpMode {


    @Override
    public void runOpMode() {
        //Na init

        initHardware(hardwareMap);

        waitForStart();
        //Na start

        /*
        Voorbeeld commando's:

        DriveForward(snelheid 0.3, afstand 200);
        Strave rechts(snelheid 0.3, afstand 200);
        Strave links(snelheid 0.3, afstand -200);
        Turn(snelheid 0.3, afstand 200);
         */

       DriveForward(0.2,140);
       Strave(0.2, 60);
       Strave(0.2, -60);
       DriveForward(0.2,70);
       Strave(0.2,60);






    }
}