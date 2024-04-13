package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class PitDownLeft extends Command {

  private ClimberSubsystem climberSubsystem;

  public PitDownLeft(ClimberSubsystem climberSubsystem) {

    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {

    climberSubsystem.pitDownLeft();
  }

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.pitStopLeft();
  }
}
