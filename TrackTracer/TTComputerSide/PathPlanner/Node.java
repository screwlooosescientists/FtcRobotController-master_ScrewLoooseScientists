package TTComputerSide.PathPlanner;

import TTComputerSide.Util.Localization.*;

public class Node {
    
  public Pose pose;
  public boolean HasCondition;
  public double Acuracy;

    public Node(Pose pose, double Acuracy, boolean HasCondition)
    {
        this.pose = pose;
        this.Acuracy = Acuracy;
        this.HasCondition = HasCondition;
    }

}
