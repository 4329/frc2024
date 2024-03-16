package frc.robot.commands.shootCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;

public class ShuffleBoardShootCommand extends Command {

  private GenericEntry rpm;
  private GenericEntry velocity;

  private ShootSubsystem shootSubsystem;

  public ShuffleBoardShootCommand(ShootSubsystem shootSubsystem) {
    this.shootSubsystem = shootSubsystem;

    rpm = Shuffleboard.getTab("shoot").add("Manuel rpm setpoint", 0).getEntry();

    addRequirements(shootSubsystem);
  }

  @Override
  public void execute() {
    shootSubsystem.setRPM(rpm.getDouble(0));
  }

  @Override
  public void end(boolean interrupted) {
    shootSubsystem.setRPM(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
