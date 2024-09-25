package frc.robot.commands.shootCommands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.AprilTagUtil;
import frc.robot.utilities.ArmAngle;
import frc.robot.utilities.MathUtils;

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

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    if (visionSubsystem.getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker())) {
      Pose3d pose3d =
          visionSubsystem.getTargetPoseInRobotSpace(
              AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker());

      System.out.println("arm shooter aim command is running");
      if (pose3d != null) {

        if (MathUtils.getActualDistanceFromPose(pose3d) < ShootSubsystem.MAX_SHOT_DISTANCE) {

          armAngleSubsystem.setArmAngle(pose3d);

        } else {

          armAngleSubsystem.setArmAngle(ArmAngle.PASS);
        }
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
