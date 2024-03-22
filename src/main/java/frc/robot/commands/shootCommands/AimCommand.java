package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;

public class AimCommand extends ParallelCommandGroup {
    
  public AimCommand(
      Drivetrain m_robotDrive,
      VisionSubsystem visionSubsystem,
      CommandXboxController driverController,
      ArmAngleSubsystem armAngleSubsystem) {

    super(
        new CenterOnTargetCommand(
            visionSubsystem,
            m_robotDrive,
            AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
            driverController),
        new ShooterAimCommand(visionSubsystem, armAngleSubsystem));
  }
}
