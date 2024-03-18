package frc.robot.commands.driveCommands;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.AprilTagUtil;
import java.util.Optional;

public class PPCenterOnTarget extends Command {
  private VisionSubsystem visionSubsystem;
  private boolean endlessStupidity;

  public PPCenterOnTarget(VisionSubsystem visionSubsystem) {
    this.visionSubsystem = visionSubsystem;
    PPHolonomicDriveController.setRotationTargetOverride(this::overrideRotation);
  }

  public Optional<Rotation2d> overrideRotation() {
    if (endlessStupidity
        && visionSubsystem.getTargetVisible(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker())) {
      Rotation2d a =
          new Rotation2d(
              visionSubsystem.faceTag(AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker()));
      return Optional.of(a);
    }
    return Optional.empty();
  }

  @Override
  public void initialize() {
    endlessStupidity = true;
  }

  @Override
  public void end(boolean interrupted) {
    endlessStupidity = false;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
