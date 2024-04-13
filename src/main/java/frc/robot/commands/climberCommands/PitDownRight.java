package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class PitDownRight extends Command {

  private ClimberSubsystem climberSubsystem;

  public PitDownRight(ClimberSubsystem climberSubsystem) {

    this.climberSubsystem = climberSubsystem;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {

    climberSubsystem.pitDownRight();
  }

  @Override
  public boolean isFinished() {

    return false;
  }

  @Override
  public void end(boolean interrupted) {
    climberSubsystem.pitStopRight();
  }
}
