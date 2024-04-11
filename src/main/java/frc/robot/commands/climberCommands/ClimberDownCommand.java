package frc.robot.commands.climberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.utilities.ArmAngle;

public class ClimberDownCommand extends Command {

  public ClimberSubsystem climberSubsystem;
  public ArmAngleSubsystem armAngleSubsystem;

  public ClimberDownCommand(
      ClimberSubsystem climberSubsystem, ArmAngleSubsystem armAngleSubsystem) {
    this.climberSubsystem = climberSubsystem;
    this.armAngleSubsystem = armAngleSubsystem;

    addRequirements(climberSubsystem, armAngleSubsystem);
  }

  @Override
  public void initialize() {
    armAngleSubsystem.setArmAngle(ArmAngle.CLIMB);
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
