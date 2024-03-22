package frc.robot.commands.intakeOuttakeCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.indexCommands.OutdexCommand;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class OutakeFull extends ParallelCommandGroup {
  public OutakeFull(IntakeSubsystem intakeSubsystem, IndexSubsystem indexSubsystem) {
    super(new OutakeCommand(intakeSubsystem), new OutdexCommand(indexSubsystem));
  }
}
