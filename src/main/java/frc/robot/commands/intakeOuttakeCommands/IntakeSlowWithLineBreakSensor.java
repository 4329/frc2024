package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmCommand;
import frc.robot.commands.indexCommands.IndexReverseForShotCommand;
import frc.robot.commands.indexCommands.IndexSensorCommand;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.utilities.ArmAngle;

public class IntakeSlowWithLineBreakSensor extends SequentialCommandGroup {
  public IntakeSlowWithLineBreakSensor(
      IntakeSubsystem intakeSubsystem,
      IndexSubsystem indexSubsystem,
      LineBreakSensorSubsystem lineBreakSensorSubsystem,
      ShootSubsystem shootSubsystem,
      ArmAngleSubsystem armAngleSubsystem) {
    super(
        new ParallelCommandGroup(
            new ArmCommand(armAngleSubsystem, ArmAngle.INTAKE),
            new IntakeSensorCommand(intakeSubsystem, lineBreakSensorSubsystem),
            new IndexSensorCommand(lineBreakSensorSubsystem, indexSubsystem)),
        new IndexReverseForShotCommand(lineBreakSensorSubsystem, indexSubsystem));
  }
}
