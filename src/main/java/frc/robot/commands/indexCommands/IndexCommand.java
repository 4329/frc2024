package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;

public class IndexCommand extends Command {

  private IndexSubsystem indexSubsystem;

  public IndexCommand(IndexSubsystem indexSubsystem) {

    this.indexSubsystem = indexSubsystem;
    addRequirements(indexSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    System.out.println("index in");
    indexSubsystem.in();
  }

  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stop();

    System.out.println("index in stop");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
