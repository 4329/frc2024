package frc.robot.commands.shootCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.AprilTagUtil;

public class DefaultShoot extends Command {

  private ShootSubsystem shootSubsystem;
  private VisionSubsystem visionSubsystem;

  public DefaultShoot(ShootSubsystem shootSubsystem, VisionSubsystem visionSubsystem) {

    this.shootSubsystem = shootSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(shootSubsystem, visionSubsystem);
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
  }

  @Override
  public void execute() {

    if (visionSubsystem.getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker())) {

      Pose3d pose3d =
          visionSubsystem.getTargetPoseInRobotSpace(
              AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());

      if (pose3d != null) {

        shootSubsystem.shooterDistance(pose3d);

      } else {

        shootSubsystem.setRPM(500);
      }

    } else {

      shootSubsystem.setRPM(500);
    }
  }

  @Override
  public void initialize() {}
}
