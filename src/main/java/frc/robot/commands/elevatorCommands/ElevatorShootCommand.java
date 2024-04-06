package frc.robot.commands.elevatorCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.ElevatorSetpoints;

public class ElevatorShootCommand extends Command {

  private ElevatorSubsystem elevatorSubsystem;
  private VisionSubsystem visionSubsystem;

  public ElevatorShootCommand(
      ElevatorSubsystem elevatorSubsystem, VisionSubsystem visionSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.visionSubsystem = visionSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {

    if (visionSubsystem.getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker())) {
      Pose3d pose3d =
          visionSubsystem.getTargetPoseInRobotSpace(
              AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());
      if (pose3d != null) {

        elevatorSubsystem.elevatorDistance(pose3d);
      }

    } else {
      elevatorSubsystem.setElevatorPose(ElevatorSetpoints.ZERO);
    }
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.atSetpoint();
  }
}
