package org.firstinspires.ftc.teamcode.classes.extra;

public class Node {

   public float X;
   public float Y;
   public float TargetHeading;
   public float acuracy;
   public boolean HasCondition;

    public Node(float X, float Y, float TargetHeading, boolean HasCondition, float accuracy)
    {
        this.X = X;
        this.Y = Y;
        this.acuracy = accuracy;
        this.TargetHeading = TargetHeading;
        this.HasCondition = HasCondition;
    }

}
