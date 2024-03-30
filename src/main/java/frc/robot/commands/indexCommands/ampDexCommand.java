package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class ampDexCommand extends Command {

  private IndexSubsystem indexSubsystem;

  public ampDexCommand(IndexSubsystem indexSubsystem) {
    this.indexSubsystem = indexSubsystem;
    addRequirements(indexSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    indexSubsystem.ampOut();
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
