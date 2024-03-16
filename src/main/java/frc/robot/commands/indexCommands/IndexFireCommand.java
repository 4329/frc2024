package frc.robot.commands.indexCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class IndexFireCommand extends Command {

  private IndexSubsystem indexSubsystem;
  private ShootSubsystem shootSubsystem;
  private Timer timer;

  public IndexFireCommand(IndexSubsystem indexSubsystem, ShootSubsystem shootSubsystem) {

    this.indexSubsystem = indexSubsystem;
    this.shootSubsystem = shootSubsystem;
    this.timer = new Timer();
    addRequirements(indexSubsystem);
  }

  @Override
  public void initialize() {

    System.out.println("index fire init");
    indexSubsystem.in();
    timer.reset();
    timer.start();
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("index fire stop");
    indexSubsystem.stop();
    shootSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    System.out.println("index fire is finished");
    return timer.hasElapsed(0.45);
  }
}
