package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utilities.ClimberSetpoints;

public class ClimberSetCommand extends Command {

  private ClimberSubsystem climberSubsystem;
  private ClimberSetpoints climberSetpoints;

  public ClimberSetCommand(ClimberSubsystem climberSubsystem, ClimberSetpoints climberSetpoints) {

    this.climberSubsystem = climberSubsystem;
    this.climberSetpoints = climberSetpoints;
    addRequirements(climberSubsystem);
  }

  @Override
  public void initialize() {

    climberSubsystem.setClimberPose(climberSetpoints);
  }

  @Override
  public boolean isFinished() {
    return climberSubsystem.atSetpoint();
  }
}
