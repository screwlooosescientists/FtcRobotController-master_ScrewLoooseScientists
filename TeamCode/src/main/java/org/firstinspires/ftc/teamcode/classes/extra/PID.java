package org.firstinspires.ftc.teamcode.classes.extra;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class PID {

    //The gains for the pid loop, kp: proportional, ki: integral, Kd: derivative
    public static double kp;
    public static double ki;
    public static double kd;

    //Target and current data,
    public double targetValue, atThisExactMomentValue;
    ElapsedTime timer = new ElapsedTime();

    public PID(double kp, double ki, double kd) //Constructor to create an pid loop :)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }


}
