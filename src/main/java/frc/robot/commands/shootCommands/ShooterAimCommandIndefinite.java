package frc.robot.commands.shootCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.ArmAngle;

public class ShooterAimCommandIndefinite extends Command {

  public VisionSubsystem visionSubsystem;
  public ArmAngleSubsystem armAngleSubsystem;
  public ElevatorSubsystem elevatorSubsystem;

  public ShooterAimCommandIndefinite(
      VisionSubsystem visionSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      ElevatorSubsystem elevatorSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
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

      System.out.println("arm shooter aim command is running");
      if (pose3d != null) {

        armAngleSubsystem.setArmAngle(pose3d);
      }
    } else {
      armAngleSubsystem.setArmAngle(ArmAngle.PASS);
    }
  }

  @Override
  public boolean isFinished() {

    return false;
  }
}
