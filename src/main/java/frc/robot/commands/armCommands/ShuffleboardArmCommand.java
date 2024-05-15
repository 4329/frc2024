package frc.robot.commands.armCommands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmAngleSubsystem;

public class ShuffleboardArmCommand extends Command {
  private ArmAngleSubsystem armAngleSubsystem;
  private GenericEntry armAngle;

  public ShuffleboardArmCommand(ArmAngleSubsystem armAngleSubsystem) {
    this.armAngleSubsystem = armAngleSubsystem;
    armAngle = Shuffleboard.getTab("shoot").add("armAngleSetpoint", 0).getEntry();
  }

  @Override
  public void initialize() {
    armAngleSubsystem.setArmAngle(armAngle.getDouble(0));
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
