package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberDownCommand extends Command {

  public ClimberSubsystem climberSubsystem;

  public ClimberDownCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  @Override
  public void execute() {

    climberSubsystem.climberDown();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
