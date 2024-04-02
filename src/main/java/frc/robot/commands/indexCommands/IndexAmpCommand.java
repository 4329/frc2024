package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;

public class IndexAmpCommand extends Command {

  private IndexSubsystem indexSubsystem;

  public IndexAmpCommand(IndexSubsystem indexSubsystem) {
    this.indexSubsystem = indexSubsystem;
    addRequirements(indexSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    indexSubsystem.backInFrontOut();
  }

  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
