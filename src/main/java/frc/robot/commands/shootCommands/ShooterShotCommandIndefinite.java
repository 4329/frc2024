package frc.robot.commands.shootCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.AprilTagUtil;

public class ShooterShotCommandIndefinite extends Command {

  private ShootSubsystem shootSubsystem;
  private VisionSubsystem visionSubsystem;

  public ShooterShotCommandIndefinite(
      ShootSubsystem shootSubsystem, VisionSubsystem visionSubsystem) {
    this.shootSubsystem = shootSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  @Override
  public void execute() {

    if (visionSubsystem.getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker())) {

      Pose3d pose3d =
          visionSubsystem.getTargetPoseInRobotSpace(
              AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());

      if (pose3d != null) {

        shootSubsystem.shooterDistance(pose3d);
      }
    }
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
