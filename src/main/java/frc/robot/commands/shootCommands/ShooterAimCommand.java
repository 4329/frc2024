package frc.robot.commands.shootCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterAimCommand extends Command {

  public VisionSubsystem visionSubsystem;
  public ArmAngleSubsystem armAngleSubsystem;

  public ShooterAimCommand(VisionSubsystem visionSubsystem, ArmAngleSubsystem armAngleSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;
    addRequirements(armAngleSubsystem);
  }

  private int getAprilTagSpeakerIDAprilTagIDSpeaker() {
    if (DriverStation.Alliance.Red.equals(DriverStation.getAlliance().get())) return 4;
    return 7;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (visionSubsystem.getTargetVisible(getAprilTagSpeakerIDAprilTagIDSpeaker())) {

      Pose3d pose3d =
          visionSubsystem.getTargetPoseInRobotSpace(getAprilTagSpeakerIDAprilTagIDSpeaker());
      if (pose3d != null) {

        armAngleSubsystem.setArmAngle(pose3d);
      }
    }
  }

  @Override
  public boolean isFinished() {

    return armAngleSubsystem.atSetpoint();
  }
}
