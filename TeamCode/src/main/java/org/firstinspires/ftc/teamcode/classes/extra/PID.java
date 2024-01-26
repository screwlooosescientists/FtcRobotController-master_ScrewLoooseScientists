package org.firstinspires.ftc.teamcode.classes.extra;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PID {

    public static double kp;
    public static double ki;
    public static double kd;
    ElapsedTime timer = new ElapsedTime();

    public PID(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }


}
