package org.firstinspires.ftc.teamcode.classes.extra;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.lang.annotation.Target;
import java.util.Timer;

public class PID {

    //The gains for the pid loop, kp: proportional, ki: integral, Kd: derivative
    public static double kp;
    public static double ki;
    public static double kd;

    //Target and current data,

    public double targetValue, currentval;
    public double LastTime;
    public double CurrentTime;

    //usefull vars
    public double lastError;


    public PID(double kp, double ki, double kd, double targetValue, double CurrentTime) //Constructor to create an pid loop :)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.targetValue = targetValue;
        this.CurrentTime = CurrentTime;
    }

    public double error()
    {
        return targetValue - currentval;
    }

    public double deltaError()
    {
        double dE = error() - lastError;
        lastError = error();
        return dE;
    }

    public double DeltaTime()
    {   double dT = CurrentTime - LastTime;
        LastTime = CurrentTime;
        return dT;
    }

    public double P()
    {
        return  error() * kp;
    }

    public double I(double currentTime)
    {
        return error() * ki * DeltaTime();
    }

    public double D(double currentTime)
    {
        return (deltaError() / DeltaTime()) * kd;
    }

    public double pidValue(double currentVal, double targetValue,  double currentTime)
    {
        this.currentval = currentVal;
        this.targetValue = targetValue;
        return P() + I(currentTime) + D(currentTime);
    }


}
