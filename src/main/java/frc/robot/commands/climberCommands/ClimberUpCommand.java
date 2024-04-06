package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCommand extends Command {

  public ClimberSubsystem climberSubsystem;

  public ClimberUpCommand(ClimberSubsystem climberSubsystem) {
    this.climberSubsystem = climberSubsystem;

    addRequirements(climberSubsystem);
  }

  @Override
  public void execute() {

    climberSubsystem.climberUp();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
