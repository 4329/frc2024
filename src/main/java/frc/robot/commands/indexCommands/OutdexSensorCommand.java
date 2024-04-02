package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;

public class OutdexSensorCommand extends Command {

  private LineBreakSensorSubsystem lineBreakSensorSubsystem;
  private IndexSubsystem indexSubsystem;
  private boolean isBroken;

  public OutdexSensorCommand(
      LineBreakSensorSubsystem lineBreakSensorSubsystem, IndexSubsystem indexSubsystem) {
    this.lineBreakSensorSubsystem = lineBreakSensorSubsystem;
    this.indexSubsystem = indexSubsystem;
    addRequirements(lineBreakSensorSubsystem, indexSubsystem);
  }

  @Override
  public void initialize() {

    this.isBroken = lineBreakSensorSubsystem.isNotBroken();
  }

  @Override
  public void execute() {

    if (lineBreakSensorSubsystem.isNotBroken()) {
      indexSubsystem.bothOut();
    } else {
      indexSubsystem.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return !lineBreakSensorSubsystem.isNotBroken();
  }

  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stop();
  }
}
