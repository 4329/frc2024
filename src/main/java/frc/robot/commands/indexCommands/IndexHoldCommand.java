package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class IndexHoldCommand extends Command {

  private IndexSubsystem indexSubsystem;

  public IndexHoldCommand(IndexSubsystem indexSubsystem) {

    this.indexSubsystem = indexSubsystem;
    addRequirements(indexSubsystem);
  }

  @Override
  public void end(boolean interrupted) {

    indexSubsystem.in();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
