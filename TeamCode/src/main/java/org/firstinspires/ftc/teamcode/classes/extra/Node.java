package org.firstinspires.ftc.teamcode.classes.extra;

public class Node {

   public float X;
   public float Y;
   public float TargetHeading;
   public boolean HasCondition;

    public Node(float X, float Y, float TargetHeading, boolean HasCondition)
    {
        this.X = X;
        this.Y = Y;
        this.TargetHeading = TargetHeading;
        this.HasCondition = HasCondition;
    }

}
