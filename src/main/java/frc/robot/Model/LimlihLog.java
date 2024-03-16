package frc.robot.Model;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class LimlihLog {
  public boolean[] tvs = new boolean[16];
  public double[] tXs = new double[16];
  public boolean limlihconnected;
  public Pose3d[] tagPoses = new Pose3d[16];

  public LimlihLog() {
    for (int i = 0; i < tagPoses.length; i++) tagPoses[i] = new Pose3d();
  }
}
