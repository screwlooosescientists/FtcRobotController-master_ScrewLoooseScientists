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
    public double i;

    public double dt;

    public PID(double kp, double ki, double kd, double targetValue, double CurrentTime) //Constructor to create an pid loop :)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.targetValue = targetValue;
        this.CurrentTime = CurrentTime;
    }

    public double error(double targetValue, double currentval)
    {
        return targetValue - currentval;
    }


    public double deltaError(double t, double c)
    {
        double dE = error(t, c) - lastError;
        lastError = error(t, c);
        return dE;
    }

    public double DeltaTime(double ct)
    {   CurrentTime = ct;
        double dT = CurrentTime - LastTime;
        LastTime = CurrentTime;

        return dT;
    }

    public double P(double t, double c)
    {
        return  error(t, c) * kp;
    }

    public double I(double t, double c, double ct)
    {
        i = i + (error(t, c) * ki * dt);
        return i;
    }

    public double D(double t, double c, double ct)
    {
        return (deltaError(t, c) / dt) * kd;
    }

    public double pidValue(double currentVal, double targetValue,  double currentTime)
    {

        double p, i , d;
        dt = DeltaTime(currentTime);
        p = P(targetValue, currentVal) ;
        i = I(targetValue, currentVal, currentTime);
        d = D(targetValue, currentVal, currentTime);
        double pid;


            pid = p + i + d;



        //return P(targetValue, currentVal) ;
        //return I(targetValue, currentVal, currentTime) ;
        return pid;
        //return P(targetValue, currentVal) + I(targetValue, currentVal, currentTime) + D(targetValue, currentVal, currentTime);
    }


}
