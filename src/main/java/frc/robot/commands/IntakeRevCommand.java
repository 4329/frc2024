package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakeOuttakeCommands.IntakeWithLineBreakSensor;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.UnInstantCommand;

public class IntakeRevCommand extends SequentialCommandGroup {
  public IntakeRevCommand(
      IntakeSubsystem intakeSubsystem,
      IndexSubsystem indexSubsystem,
      LineBreakSensorSubsystem lineBreakSensorSubsystem,
      ArmAngleSubsystem armAngleSubsystem,
      ShootSubsystem shootSubsystem) {
    super(
        new IntakeWithLineBreakSensor(
            intakeSubsystem, indexSubsystem, lineBreakSensorSubsystem, armAngleSubsystem),
        new UnInstantCommand((/*portal thingy*/ ) -> shootSubsystem.changeSetpoint(2000)));
  }
}
