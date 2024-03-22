package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexCommands.IndexCommand;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFull extends ParallelCommandGroup {
  public IntakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
    super(new IntakeCommand(intakeSubsystem), new IndexCommand(indexSubsystem));
  }
}
