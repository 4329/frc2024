package frc.robot.commands.shootCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.AprilTagUtil;

public class ShooterShotCommand extends Command {

  private ShootSubsystem shootSubsystem;
  private IndexSubsystem indexSubsystem;
  private VisionSubsystem visionSubsystem;
  private Timer timer = new Timer();
  private boolean shot = false;

  public ShooterShotCommand(
      ShootSubsystem shootSubsystem,
      IndexSubsystem indexSubsystem,
      VisionSubsystem visionSubsystem) {
    this.shootSubsystem = shootSubsystem;
    this.indexSubsystem = indexSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(shootSubsystem);
  }

  @Override
  public void initialize() {

    shot = false;
    timer.reset();

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
  public void execute() {

    System.out.println("is shooting from shootershot");
    if (!shot && shootSubsystem.aboveSetpoint() == true) {

      timer.start();
      indexSubsystem.in();
      shot = true;
    }
  }

  @Override
  public boolean isFinished() {

    return timer.hasElapsed(0.4);
  }

  @Override
  public void end(boolean interrupted) {

    shootSubsystem.stop();
    indexSubsystem.stop();
    timer.stop();
  }
}
