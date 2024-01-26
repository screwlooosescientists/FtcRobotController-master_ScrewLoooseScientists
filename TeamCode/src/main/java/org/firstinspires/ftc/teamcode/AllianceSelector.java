package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.classes.extra.StartingPositions;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

@TeleOp(name = "Select Aliance", group = "Centerstage")
public class AllianceSelector extends LinearOpMode{

    private StartingPositions.StartPos sPos;

    @Override
    public void runOpMode() {
        telemetry.addData("Status: ", "Selecting start pos...");
        telemetry.addData("StartPos: ", sPos);
        telemetry.update();

        //Autonomous selecting pos
        while(!gamepad1.back && !isStopRequested())
        {
            if (gamepad1.a)
                sPos = StartingPositions.StartPos.BLUE1;

            if (gamepad1.b)
                sPos = StartingPositions.StartPos.BLUE2;

            if (gamepad1.x)
                sPos = StartingPositions.StartPos.RED1;

            if (gamepad1.y)
                sPos = StartingPositions.StartPos.RED2;

            telemetry.addData("Status: ", "Selecting start pos...");
            telemetry.addData("StartPos: ", sPos);
            telemetry.update();

        }

        waitForStart();

        try {
            File startposFile = new File("\\org\\firstinspires\\ftc\\teamcode\\classes\\extra\\datafiles\\StartingPosition.txt");
            Writer fileWriter = new FileWriter(startposFile, false);
            fileWriter.write(sPos.name());

        }
        catch (IOException e)
        {
            telemetry.addData("Error: ", e);
            telemetry.update();
        }


    }
}
