package frc.robot.commands.shootCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.OutdexSensorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.UnInstantCommand;

public class ShooterSourceCommand extends SequentialCommandGroup {

  public ShooterSourceCommand(
      IndexSubsystem indexSubsystem,
      ShootSubsystem shootSubsystem,
      LineBreakSensorSubsystem lineBreakSensorSubsystem,
      ArmAngleSubsystem armAngleSubsystem) {

    super(

        // new ArmAngleCommand(armAngleSubsystem, ArmAngle.SHOOTERSOURCE),
        new UnInstantCommand(() -> shootSubsystem.changeSetpoint(-750)),
        new OutdexSensorCommand(lineBreakSensorSubsystem, indexSubsystem),
        new UnInstantCommand(() -> shootSubsystem.changeSetpoint(-500)),
        new IndexReverseForShotCommand(lineBreakSensorSubsystem, indexSubsystem),
        new UnInstantCommand(() -> shootSubsystem.changeSetpoint(0)));
  }
}
