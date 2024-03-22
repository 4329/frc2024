package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.driveCommands.CenterOnTargetCommand;
import frc.robot.commands.indexCommands.IndexFireCommand;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swerve.Drivetrain;
import frc.robot.utilities.AprilTagUtil;

public class CenterAndFireCommand extends SequentialCommandGroup {

  public CenterAndFireCommand(
      VisionSubsystem visionSubsystem,
      Drivetrain drivetrain,
      IndexSubsystem indexSubsystem,
      ShootSubsystem shootSubsystem,
      CommandXboxController commandXboxController) {

    super(
        new CenterOnTargetCommand(
            visionSubsystem,
            drivetrain,
            AprilTagUtil.getAprilTagSpeakerIDAprilTagIDSpeaker(),
            commandXboxController),
        new IndexFireCommand(indexSubsystem, shootSubsystem));
  }
}
