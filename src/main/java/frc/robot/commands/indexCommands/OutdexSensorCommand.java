package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LineBreakSensorSubsystem;

public class OutdexSensorCommand extends Command {

  private LineBreakSensorSubsystem lineBreakSensorSubsystem;
  private IndexSubsystem indexSubsystem;
  private boolean isBroken;
  private Timer timer;

  public OutdexSensorCommand(
      LineBreakSensorSubsystem lineBreakSensorSubsystem, IndexSubsystem indexSubsystem) {
    this.lineBreakSensorSubsystem = lineBreakSensorSubsystem;
    this.indexSubsystem = indexSubsystem;
    this.timer = new Timer();
    addRequirements(lineBreakSensorSubsystem, indexSubsystem);
  }

  @Override
  public void initialize() {

    this.isBroken = lineBreakSensorSubsystem.isNotBroken();
    timer.reset();
  }

  @Override
  public void execute() {

    if (lineBreakSensorSubsystem.isNotBroken()) {
      indexSubsystem.bothOut();
      timer.reset();
    } else {
      timer.start();
    }
  }

  @Override
  public boolean isFinished() {
    return !lineBreakSensorSubsystem.isNotBroken() && timer.hasElapsed(0.06);
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
    indexSubsystem.stop();
  }
}
