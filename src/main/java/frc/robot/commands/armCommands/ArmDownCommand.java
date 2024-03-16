package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;

public class ArmDownCommand extends Command {

  public ArmAngleSubsystem armAngleSubsystem;

  public ArmDownCommand(ArmAngleSubsystem armAngleSubsystem) {
    this.armAngleSubsystem = armAngleSubsystem;

    addRequirements(armAngleSubsystem);
  }

  @Override
  public void execute() {

    armAngleSubsystem.armPositonDown();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
