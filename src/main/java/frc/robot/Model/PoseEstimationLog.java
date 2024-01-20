package frc.robot.Model;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

@AutoLog
public class PoseEstimationLog {
    public Pose2d combined;
    public Pose2d limOnly;
    public Pose2d driveOnly;
}
